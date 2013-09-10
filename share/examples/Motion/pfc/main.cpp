#include <Core/util.h>
#include <Motion/motion.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/optimization_benchmarks.h>
#include "mobject.h"
#include <vector>
#include <GL/glu.h>
#include <stdlib.h>

const double PI = 3.1415926535897932384626;

void moveObject(const char* name, MotionProblem& _P, double t, double T, double speed){
  _P.ors->getBodyByName(name)->X.pos = _P.ors->getBodyByName("trainTargetOrig")->X.pos + ARRAY(0.5*sin(0.5*PI*(speed*t)/T), 0.*sin(speed*t/T),0.);
}

void moveObjectLinear(const char* name, MotionProblem& _P, const arr&  dir, double step){
  _P.ors->getBodyByName(name)->X.pos = _P.ors->getBodyByName(name)->X.pos + (dir*step);
}

void pfcPosition() {
  OpenGL gl;
  ors::Graph G;
  uint t;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  MotionProblem P(&G);
  P.loadTransitionParameters();

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addDefaultTaskMap_Bodies("position", posTMT,"endeff",ors::Transformation().setText("<t(0 0 0)>"));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(P.ors->getBodyByName("trainTarget")->X.pos), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);

  c = P.addDefaultTaskMap_Bodies("orientation", zoriTMT,"endeff",ors::Transformation().setText("<t(0 0 0)>"));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(0.,0.,-1.), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.get_n();
  cout <<"Problem parameters:"<<"\n T=" <<T<<"\n k=" <<k<<"\n n=" <<n<<"\n # joints=" <<G.getJointStateDimension()<<endl;

  //mini evaluation test:
  arr x(T+1,n);
  x.setZero();

  //-- optimize
  optGaussNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));

  P.costReport();
  write(LIST<arr>(x),"output/z.output");
  displayTrajectory(x, 1, G, gl,"planned trajectory");

  //-- save optimal trajectory in task space
  arr kinPos, kinVec, xRefPos, xRefVec;
  // store cartesian coordinates and rotation vector
  for (t=0;t<=T;t++) {
    G.setJointState(x[t]);
    G.calcBodyFramesFromJoints();
    G.kinematicsPos(kinPos,P.ors->getBodyByName("endeff")->index);
    G.kinematicsVec(kinVec,P.ors->getBodyByName("endeff")->index);
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  // transform trajectory to task space (\phi(q) - objTrain)
  arr fProjPos, fProjVec, objTrainPos, objTrainVec;
  G.kinematicsPos(objTrainPos, G.getBodyByName("trainTarget")->index);
  G.kinematicsVec(objTrainVec, G.getBodyByName("trainTarget")->index);

  double s=0.0;
  for (t=0;t<=T;t++) {
    s = (double)t/T;
    fProjPos.append(~(xRefPos[t] - s*objTrainPos));
    fProjVec.append(~(xRefVec[t] - s*objTrainVec));
  }


  //-- replay trajectory with same target position
  arr x0; G.getJointState(x0); x0.setZero();

  arr yPos, yVec, fPos_target, fVec_target, JPos, JVec, Phi, PhiJ, q, W, targetPos, targetVec;
  arr idxRec, xRecPos, xRecVec, oRecPos, oRecVec, errRecPos, errRecVec;
  arr fPos, fVec;
  arr curr, prev, next;
  arr fRecPos,fRecVec, fProjPosRec, fProjVecRec;

  double dc,dn,dp;
  double fPos_deviation, fVec_deviation;
  uint idx=0;

  G.setJointState(x0,x0);
  G.calcBodyFramesFromJoints();
  G.getJointState(q);

  W.setDiag(1.,G.getJointStateDimension());  // W is equal the Id_n matrix
  W = W*100.;

  fPos_deviation = 1e-1;
  fVec_deviation = 1e-1;

  s=0.0;

  while (idx < T) {
    // Compute current task coordinate
    G.kinematicsVec(targetVec, G.getBodyByName("testTarget")->index);
    G.kinematicsVec(yVec, G.getBodyByName("endeff")->index);
    G.jacobianVec(JVec, G.getBodyByName("endeff")->index);

    G.kinematicsPos(targetPos, G.getBodyByName("testTarget")->index);
    G.kinematicsPos(yPos, G.getBodyByName("endeff")->index);
    G.jacobianPos(JPos, G.getBodyByName("endeff")->index);

    s = (double)idx/T;
    fPos = yPos - targetPos*s;
    fVec = yVec - targetVec*s;

    // find current position in plan and next target
    idx++;
    fVec_target = fProjVec[idx];
    fPos_target = fProjPos[idx];

    // 1st task: DIRECTION
    Phi = ((fVec - fVec_target)/ fVec_deviation);
    PhiJ = (JVec / fVec_deviation);

    // 2nd task: POSITION
    Phi.append((fPos - fPos_target)/ fPos_deviation);
    PhiJ.append(JPos / fPos_deviation);

    // bookkeeping
    errRecPos.append(norm(fPos-fPos_target));
    errRecVec.append(norm(fVec - fVec_target));
    fRecPos.append(~fPos);
    fProjPosRec.append(~fProjPos[idx]);
    fProjVecRec.append(~fProjVec[idx]);
    fRecVec.append(~fVec);
    xRecPos.append(~yPos);
    xRecVec.append(~yVec);
    oRecPos.append(~targetPos);
    oRecVec.append(~targetVec);
    idxRec.append(idx);

    // compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;

    // sets joint angles AND computes all frames AND updates display
    G.setJointState(q);
    G.calcBodyFramesFromJoints();

    moveObjectLinear("testTarget",P,ARRAY(-1.,-1.,0.),0.003);

    gl.update();
  }

  //-- PLOTS
  {
    gnuplot("bind 1 'set term wxt 1; raise 1'");
    gnuplot("bind 2 'set term wxt 2; raise 2'");
    gnuplot("bind 3 'set term wxt 3; raise 3'");
    gnuplot("bind 4 'set term wxt 4; raise 4'");
    gnuplot("bind 5 'set term wxt 5; raise 5'");
    gnuplot("bind 6 'set term wxt 6; raise 6'");
    gnuplot("bind 7 'set term wxt 7; raise 7'");
    gnuplot("bind 8 'set term wxt 8; raise 8'");
    gnuplot("bind 9 'set term wxt 9; raise 9'");

    write(LIST<arr>(xRefPos),"out/xRefPos.output");
    write(LIST<arr>(xRecPos),"out/xRecPos.output");
    write(LIST<arr>(oRecPos),"out/oRecPos.output");

    gnuplot("set term wxt 1");
    gnuplot("plot 'out/xRefPos.output' us 1,'out/xRecPos.output' us 1, 'out/oRecPos.output' us 1");
    gnuplot("set term wxt 2");
    gnuplot("plot 'out/xRefPos.output' us 2,'out/xRecPos.output' us 2, 'out/oRecPos.output' us 2");
    gnuplot("set term wxt 3");
    gnuplot("plot 'out/xRefPos.output' us 3,'out/xRecPos.output' us 3, 'out/oRecPos.output' us 3");

    //      write(LIST<arr>(xRefVec),"out/xRefVec.output");
    //      write(LIST<arr>(xRecVec),"out/xRecVec.output");
    //      write(LIST<arr>(oRecVec),"out/oRecVec.output");
    //      gnuplot("set term wxt 4");
    //      gnuplot("plot 'out/xRefVec.output' us 1,'out/xRecVec.output' us 1, 'out/oRecVec.output' us 1");
    //      gnuplot("set term wxt 5");
    //      gnuplot("plot 'out/xRefVec.output' us 2,'out/xRecVec.output' us 2, 'out/oRecVec.output' us 2");
    //      gnuplot("set term wxt 6");
    //      gnuplot("plot 'out/xRefVec.output' us 3,'out/xRecVec.output' us 3, 'out/oRecVec.output' us 3");

    write(LIST<arr>(errRecPos),"out/errRecPos.output");
    write(LIST<arr>(errRecVec),"out/errRecVec.output");
    gnuplot("set term wxt 7");
    gnuplot("plot 'out/errRecPos.output' us 1, 'out/errRecVec.output' us 1");

    write(LIST<arr>(fRecPos),"out/fRecPos.output");
    write(LIST<arr>(fProjPosRec),"out/fProjPosRec.output");
    gnuplot("set term wxt 8");
    gnuplot("plot 'out/fRecPos.output' us 1, 'out/fProjPosRec.output' us 1");
    gnuplot("set term wxt 9");
    gnuplot("plot 'out/fRecPos.output' us 2, 'out/fProjPosRec.output' us 2");
    gnuplot("set term wxt 10");
    gnuplot("plot 'out/fRecPos.output' us 3, 'out/fProjPosRec.output' us 3");

    //      gnuplot("set term wxt 99",false, true);
    //      gnuplot(idxRec);
  }

  gl.watch();
}


void drawEnv(void* classP){
  MObject *mo=(MObject*)classP;

//  double GLmatrix[16];
//  ors::Transformation f;
//  f.setZero();
//  f.addRelativeTranslation(mo->prediction(0),mo->prediction(1),mo->prediction(2));
//  f.getAffineMatrixGL(GLmatrix);
//  glLoadMatrixd(GLmatrix);
//  glDrawSphere(0.03);

  glColor(5);
  glLineWidth(5);
  glBegin(GL_LINES);
  glVertex3f(mo->position(0),mo->position(1),mo->position(2));
  glVertex3f(mo->prediction(0),mo->prediction(1),mo->prediction(2));
  glEnd();
  glLineWidth(1);
}

void pfcObstacle() {
  OpenGL gl;
  ors::Graph G;
  uint t;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));



  MotionProblem P(&G);
  P.loadTransitionParameters();

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addDefaultTaskMap_Bodies("position", posTMT,"endeff",ors::Transformation().setText("<t(0 0 0)>"));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(P.ors->getBodyByName("trainTarget")->X.pos), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);

  c = P.addDefaultTaskMap_Bodies("orientation", zoriTMT,"endeff",ors::Transformation().setText("<t(0 0 0)>"));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(0.,0.,-1.), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);

  c = P.addDefaultTaskMap("collision", collTMT, 0, Transformation_Id, 0, Transformation_Id, ARR(.1));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e0);

  // Create goal and obstacles vector
  std::vector<MObject*> mobjects;
  mobjects.push_back(new MObject(&G, MT::String("obstacle1"), MObject::OBSTACLE , 0.002, ARRAY(1.,0.,0.)));
  mobjects.push_back(new MObject(&G, MT::String("obstacle2"), MObject::OBSTACLE , 0.001, ARRAY(0.,1.,0.)));
  mobjects.push_back(new MObject(&G, MT::String("obstacle3"), MObject::OBSTACLE , 0.001, ARRAY(0.,0.,1.)));
  mobjects.push_back(new MObject(&G, MT::String("testTarget"), MObject::GOAL, 0., ARRAY(0.,0.,0.)));

  gl.add(drawEnv,mobjects.at(0));
  gl.add(drawEnv,mobjects.at(1));
  gl.add(drawEnv,mobjects.at(2));
  gl.add(drawEnv,mobjects.at(3));



  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.get_n();
  cout <<"Problem parameters:"<<"\n T=" <<T<<"\n k=" <<k<<"\n n=" <<n<<"\n # joints=" <<G.getJointStateDimension()<<endl;

  //mini evaluation test:
  arr x(T+1,n);
  x.setZero();

  //-- optimize
  optGaussNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));

  P.costReport();
  write(LIST<arr>(x),"output/z.output");
  //  displayTrajectory(x, 1, G, gl,"planned trajectory");

  //-- save optimal trajectory in task space
  arr kinPos, kinVec, xRefPos, xRefVec;
  // store cartesian coordinates and rotation vector
  for (t=0;t<=T;t++) {
    G.setJointState(x[t]);
    G.calcBodyFramesFromJoints();
    G.kinematicsPos(kinPos,P.ors->getBodyByName("endeff")->index);
    G.kinematicsVec(kinVec,P.ors->getBodyByName("endeff")->index);
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  // transform trajectory to task space (\phi(q) - objTrain)
  arr fProjPos, fProjVec, fProjObs, objTrainPos, objTrainVec, obsTrainPos;
  G.kinematicsPos(objTrainPos, G.getBodyByName("trainTarget")->index);
  G.kinematicsVec(objTrainVec, G.getBodyByName("trainTarget")->index);

  G.kinematicsPos(obsTrainPos, G.getBodyByName("obstacle")->index);

  cout << "obsTrainPos: " << obsTrainPos << endl;
  double s=0.0;
  for (t=0;t<=T;t++) {
    s = (double)t/T;
    fProjPos.append(~(xRefPos[t] - s*objTrainPos));
    fProjVec.append(~(xRefVec[t] - s*objTrainVec));
    fProjObs.append(~(xRefPos[t] - s*obsTrainPos));
  }


  //-- replay trajectory with same target position
  arr x0; G.getJointState(x0); x0.setZero();

  arr yPos, yVec, fPos_target, fVec_target, JPos, JVec, Phi, PhiJ, q, W, targetPos, targetVec;
  arr idxRec, xRecPos, xRecVec, oRecPos, oRecVec, errRecPos, errRecVec;
  arr fPos, fVec;
  arr fRecPos,fRecVec, fProjPosRec, fProjVecRec;
  arr fObs,fObs_target,obsPos;
  arr posCosts,rotCosts,obsCosts;

  double dc,dn,dp;
  double fPos_deviation, fVec_deviation, fObs_deviation;
  uint idx=0;
  double eps_act = 0.9;
  double eps_lim= 0.3;
  double coll_dist;
  double obs_cost;

  G.setJointState(x0,x0);
  G.calcBodyFramesFromJoints();
  G.getJointState(q);

  W.setDiag(1.,G.getJointStateDimension());  // W is equal the Id_n matrix
  W = W*100.;

  fPos_deviation = 1e-1;
  fVec_deviation = 1e-1;
  fObs_deviation = 1e-1;

  s=0.0;

  while (idx < T) {
    // Compute current task coordinate
    G.kinematicsVec(targetVec, G.getBodyByName("testTarget")->index);
    G.kinematicsVec(yVec, G.getBodyByName("endeff")->index);
    G.jacobianVec(JVec, G.getBodyByName("endeff")->index);

    G.kinematicsPos(targetPos, G.getBodyByName("testTarget")->index);
    G.kinematicsPos(yPos, G.getBodyByName("endeff")->index);
    G.jacobianPos(JPos, G.getBodyByName("endeff")->index);


    s = (double)idx/T;
    fPos = yPos - targetPos*s;
    fVec = yVec - targetVec*s;

    // find current position in plan and next target
    idx++;
    fVec_target = fProjVec[idx];
    fPos_target = fProjPos[idx];

    // 1st task: DIRECTION
    rotCosts.append(sum((fVec - fVec_target)/ fVec_deviation));
    Phi = ((fVec - fVec_target)/ fVec_deviation);
    PhiJ = (JVec / fVec_deviation);

    // 2nd task: POSITION
    posCosts.append(sum((fPos - fPos_target)/ fPos_deviation));
    Phi.append((fPos - fPos_target)/ fPos_deviation);
    PhiJ.append(JPos / fPos_deviation);

    // 3rd task: OBSTACLE
    double total_obs_cost = 0.;
    for (std::vector<MObject*>::iterator moIter = mobjects.begin() ; moIter != mobjects.end() ; ++moIter) {
      if ((*moIter)->objectType == MObject::OBSTACLE) {
        (*moIter)->pose(obsPos);

        fObs = yPos - obsPos;
        coll_dist = norm(fObs);
        obs_cost = 0.;
        if (coll_dist < eps_act) {
          // log costs
          //              obs_cost = (- log(coll_dist-eps_lim) + log(eps_act-eps_lim) ) / fObs_deviation;
          //              Phi.append( obs_cost );
          //              PhiJ.append( ( (~fObs*JPos) / (coll_dist*(eps_lim - coll_dist)) ) / fObs_deviation);

          // linear costs
          double b = 1;
          obs_cost = ((-b/eps_act)* coll_dist + b) / fObs_deviation;
          Phi.append( obs_cost );
          PhiJ.append( ((-b/eps_act)*(~fObs*JPos)/coll_dist) / fObs_deviation);

          // squared costs
          double c = 0.5;
          //        obs_cost = ((c/(eps_act*eps_act))* coll_dist*coll_dist - 2*(c/eps_act)*coll_dist + c) / fObs_deviation;
          //        Phi.append( obs_cost );
          //        PhiJ.append(  ((2*c/(eps_act*eps_act))*(~fObs*JPos) + (-2*c/eps_act)*(~fObs*JPos)/coll_dist) / fObs_deviation);

          //      cout << "Distance to object: " << coll_dist << endl;
          total_obs_cost += obs_cost;
        }
      }
    }

    obsCosts.append(total_obs_cost);

    // move obstacles
    for (std::vector<MObject*>::iterator moIter = mobjects.begin() ; moIter != mobjects.end() ; ++moIter) {
      (*moIter)->move();
      (*moIter)->predict(T-idx);
    }


    // bookkeeping
    errRecPos.append(norm(fPos-fPos_target));
    errRecVec.append(norm(fVec - fVec_target));
    fRecPos.append(~fPos);
    fProjPosRec.append(~fProjPos[idx]);
    fProjVecRec.append(~fProjVec[idx]);
    fRecVec.append(~fVec);
    xRecPos.append(~yPos);
    xRecVec.append(~yVec);
    oRecPos.append(~targetPos);
    oRecVec.append(~targetVec);
    idxRec.append(idx);

    // compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;

    // sets joint angles AND computes all frames AND updates display
    G.setJointState(q);
    G.calcBodyFramesFromJoints();

    gl.update();
  }


  //-- PLOTS
  {
    gnuplot("bind 1 'set term wxt 1; raise 1'");
    gnuplot("bind 2 'set term wxt 2; raise 2'");
    gnuplot("bind 3 'set term wxt 3; raise 3'");
    gnuplot("bind 4 'set term wxt 4; raise 4'");
    gnuplot("bind 5 'set term wxt 5; raise 5'");
    gnuplot("bind 6 'set term wxt 6; raise 6'");
    gnuplot("bind 7 'set term wxt 7; raise 7'");
    gnuplot("bind 8 'set term wxt 8; raise 8'");
    gnuplot("bind 9 'set term wxt 9; raise 9'");

    write(LIST<arr>(xRefPos),"out/xRefPos.output");
    write(LIST<arr>(xRecPos),"out/xRecPos.output");
    write(LIST<arr>(oRecPos),"out/oRecPos.output");

    gnuplot("set term wxt 1");
    gnuplot("plot 'out/xRefPos.output' us 1,'out/xRecPos.output' us 1, 'out/oRecPos.output' us 1");
    gnuplot("set term wxt 2");
    gnuplot("plot 'out/xRefPos.output' us 2,'out/xRecPos.output' us 2, 'out/oRecPos.output' us 2");
    gnuplot("set term wxt 3");
    gnuplot("plot 'out/xRefPos.output' us 3,'out/xRecPos.output' us 3, 'out/oRecPos.output' us 3");

    write(LIST<arr>(posCosts),"out/posCosts.output");
    write(LIST<arr>(rotCosts),"out/rotCosts.output");
    write(LIST<arr>(obsCosts),"out/obsCosts.output");

    gnuplot("set term wxt 4");
    gnuplot("plot 'out/posCosts.output' us 1,'out/rotCosts.output' us 1, 'out/obsCosts.output' us 1");

    //      write(LIST<arr>(xRefVec),"out/xRefVec.output");
    //      write(LIST<arr>(xRecVec),"out/xRecVec.output");
    //      write(LIST<arr>(oRecVec),"out/oRecVec.output");
    //      gnuplot("set term wxt 4");
    //      gnuplot("plot 'out/xRefVec.output' us 1,'out/xRecVec.output' us 1, 'out/oRecVec.output' us 1");
    //      gnuplot("set term wxt 5");
    //      gnuplot("plot 'out/xRefVec.output' us 2,'out/xRecVec.output' us 2, 'out/oRecVec.output' us 2");
    //      gnuplot("set term wxt 6");
    //      gnuplot("plot 'out/xRefVec.output' us 3,'out/xRecVec.output' us 3, 'out/oRecVec.output' us 3");

    write(LIST<arr>(errRecPos),"out/errRecPos.output");
    write(LIST<arr>(errRecVec),"out/errRecVec.output");
    gnuplot("set term wxt 7");
    gnuplot("plot 'out/errRecPos.output' us 1, 'out/errRecVec.output' us 1");

    //    write(LIST<arr>(fRecPos),"out/fRecPos.output");
    //    write(LIST<arr>(fProjPosRec),"out/fProjPosRec.output");
    //    gnuplot("set term wxt 8");
    //    gnuplot("plot 'out/fRecPos.output' us 1, 'out/fProjPosRec.output' us 1");
    //    gnuplot("set term wxt 9");
    //    gnuplot("plot 'out/fRecPos.output' us 2, 'out/fProjPosRec.output' us 2");
    //    gnuplot("set term wxt 10");
    //    gnuplot("plot 'out/fRecPos.output' us 3, 'out/fProjPosRec.output' us 3");

    //      gnuplot("set term wxt 99",false, true);
    //      gnuplot(idxRec);
  }

  gl.watch();
}


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  switch(MT::getParameter<int>("mode",2)){
  case 1:  pfcPosition();  break;
  case 2:  pfcObstacle();  break;
  }

  return 0;
}
