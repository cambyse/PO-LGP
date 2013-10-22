#include <Core/util.h>
#include <Motion/motion.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include "mobject.h"
#include "pfc.h"
#include <vector>
#include <GL/glu.h>
#include <stdlib.h>

const double PI = 3.1415926535897932384626;

void drawEnv(void* classP){
  MObject *mo=(MObject*)classP;
  glColor(5);
  glLineWidth(1);
  glBegin(GL_LINES);
  glVertex3f(mo->position(0),mo->position(1),mo->position(2));
  glEnd();
  glLineWidth(1);
}

void drawTraj(uint color, arr& p, uint lineStyle) {
  glColor(color);
  glPointSize(4.0f);
  glLineWidth(2);
  if (lineStyle == 1) {
    glBegin(GL_POINTS);
  } else {
    glBegin(GL_LINES);
  }
  glVertex3f(p(0,0),p(0,1),p(0,2));
  uint i;
  for (i = 1; i<p.d0-1; i++) {
    glVertex3f(p(i,0),p(i,1),p(i,2));
    glVertex3f(p(i,0),p(i,1),p(i,2));
  }
  glVertex3f(p(i,0),p(i,1),p(i,2));
  glEnd();
  glLineWidth(1);
}

void drawActTraj(void* classP){
  arr *p = (arr*)classP;
  drawTraj(5,*p,1);
}
void drawPlanTraj(void* classP){
  arr *p = (arr*)classP;
  drawTraj(1,*p,2);
}
void drawRefTraj(void* classP){
  arr *p = (arr*)classP;
  drawTraj(2,*p,1);
}


void runPFC(String scene, bool useOrientation, bool useCollAvoid) {
  //------------------------------------------------//
  // Compute optimal trajectory
  //------------------------------------------------//
  OpenGL gl(scene,800,800);
  ors::Graph G;
  init(G, gl, scene);

  MotionProblem P(&G);
  P.loadTransitionParameters();
//  P.swift->deactivate(P.ors->getBodyByName("obstacle1")->shapes(0),P.ors->getBodyByName("obstacle2")->shapes(0));

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addDefaultTaskMap_Bodies("position", posTMT,"endeff",ors::Transformation().setText("<t(0 0 0)>"));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(P.ors->getBodyByName("goalRef")->X.pos), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);

  if (useOrientation) {
    c = P.addDefaultTaskMap_Bodies("orientation", zoriTMT,"endeff",ors::Transformation().setText("<t(0 0 0)>"));
    P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                            ARRAY(0.,0.,-1.), 1e4,
                            ARRAY(0.,0.,0.), 1e-3);
  }

  if (useCollAvoid) {
    c = P.addDefaultTaskMap("collision", collTMT, 0, Transformation_Id, 0, Transformation_Id, ARR(.1));
    P.setInterpolatingCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e0);
  }

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();

  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n<<" # joints=" <<G.getJointStateDimension()<<endl;

  //-- mini evaluation test:
  arr x(T+1,n);
  x.setZero();

  //-- optimize
  optGaussNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));

  P.costReport();
  //  displayTrajectory(x, 1, G, gl,"planned trajectory");

  //------------------------------------------------//
  // Transform trajectory into task space
  //------------------------------------------------//
  arr kinPos, kinVec, xRefPos, xRefVec, xRef;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    G.setJointState(x[t]);
    G.calcBodyFramesFromJoints();
    G.kinematicsPos(kinPos,P.ors->getBodyByName("endeff")->index);
    G.kinematicsVec(kinVec,P.ors->getBodyByName("endeff")->index);
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  xRef = xRefPos;
  if (useOrientation) {
    xRef = ~cat(~xRef,~xRefVec);
  }

  //------------------------------------------------//
  // Create obstacles and goals
  //------------------------------------------------//
  MObject goalMO(&G, MT::String("goal"), MObject::GOAL , 0.00, ARRAY(0.,1.,0.));
  std::vector<MObject*> mobjects;

  //    mobjects.push_back(new MObject(&G, MT::String("obstacle1"), MObject::OBSTACLE , 0.003, ARRAY(0.,0.,-1.)));
  //    gl.add(drawEnv,mobjects.at(0));

  //------------------------------------------------//
  // Create pfc from optimal trajectory in task space
  //------------------------------------------------//
  arr x0 = xRef[0];
  Pfc *pfc = new Pfc(xRef,2.,x0, goalMO, useOrientation);
  gl.add(drawActTraj,&(pfc->traj));
  gl.add(drawRefTraj,&(pfc->trajRef->points));
  gl.add(drawPlanTraj,&(pfc->trajWrap->points));

  //------------------------------------------------//
  // Simulate controller
  //------------------------------------------------//
  arr W, yPos, JPos, yVec, JVec, yPos_target,yVec_target, y_target, q, Phi, PhiJ, q0, dq0,yCol,JCol,costs,posCosts,vecCosts,colCosts;
  double fPos_deviation = 1e-1;
  double fVec_deviation = 1e-1;
  double yCol_deviation = 3e-1;

  W.setDiag(1.,G.getJointStateDimension());  // W is equal the Id_n matrix
  W = W*10.;

  q0 = x[0]; dq0 = 0.*q0;
  G.setJointState(q0,dq0);
  G.calcBodyFramesFromJoints();
  G.getJointState(q);

  uint t = 0;
  while((pfc->s.last()<0.99) && t++ < 2*T)
  {
    // Compute current task states
    G.kinematicsPos(yPos, G.getBodyByName("endeff")->index);
    G.jacobianPos(JPos, G.getBodyByName("endeff")->index);
    G.kinematicsVec(yVec, G.getBodyByName("endeff")->index);
    G.jacobianVec(JVec, G.getBodyByName("endeff")->index);

    // move obstacles
    for (std::vector<MObject*>::iterator moIter = mobjects.begin() ; moIter != mobjects.end() ; ++moIter) {
      (*moIter)->move();
    }

    // move goal
    goalMO.move();

    // iterate pfc
    arr y = yPos;
    if (useOrientation) {
      y.append(yVec);
    }
    pfc->iterate(y);

    // find current position in plan and next target
    y_target = pfc->traj[pfc->traj.d0-1];
    yPos_target = y_target.subRange(0,2);


    // task 1: POSITION
    costs = (yPos - yPos_target)/ fPos_deviation;
    posCosts.append(~costs*costs);
    Phi = ((yPos - yPos_target)/ fPos_deviation);
    PhiJ = (JPos / fPos_deviation);

    // task 2: ORIENTATION
    if (useOrientation) {
      yVec_target = y_target.subRange(3,5);
      costs = (yVec - yVec_target)/ fVec_deviation;
      vecCosts.append(~costs*costs);
      Phi.append(costs);
      PhiJ.append(JVec / fVec_deviation);
    }

    // task 3: COLLISION
    if (useCollAvoid) {
      P.swift->computeProxies(G);
      G.phiCollision(yCol,JCol,0.15);
      costs = yCol / yCol_deviation;
      colCosts.append(~costs*costs);
      Phi.append(costs);
      PhiJ.append(JCol / yCol_deviation);
    }

    // compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;

    // sets joint angles AND computes all frames AND updates display
    G.setJointState(q);
    G.calcBodyFramesFromJoints();

    gl.update();
  }

  //------------------------------------------------//
  // Plot results
  //------------------------------------------------//
  pfc->plotState();
  gnuplot("set term wxt 21 title 'cost overview'");
  write(LIST<arr>(posCosts),"out/posCosts.output");
  gnuplot("plot 'out/posCosts.output' us 1");

  if (useOrientation) {
    write(LIST<arr>(vecCosts),"out/vecCosts.output");
     gnuplot("replot 'out/vecCosts.output' us 1");
  }
  if (useCollAvoid) {
    write(LIST<arr>(colCosts),"out/colCosts.output");
      gnuplot("replot 'out/colCosts.output' us 1");
  }

  gl.watch();
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  runPFC(String("scenes/scene1.ors"),true,false);
  runPFC(String("scenes/scene2.ors"),false,false);
  runPFC(String("scenes/scene3.ors"),false,true);
  runPFC(String("scenes/scene4.ors"),true,true);
  runPFC(String("scenes/scene5.ors"),true,true);
  runPFC(String("scenes/scene6.ors"),true,true);
  runPFC(String("scenes/scene7.ors"),true,true);
  runPFC(String("scenes/scene8.ors"),false,false);
  runPFC(String("scenes/scene9.ors"),false,true);

  return 0;
}
