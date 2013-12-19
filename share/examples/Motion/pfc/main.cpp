#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
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
  ors::KinematicWorld G(scene);
  G.gl().resize(800,800);

  makeConvexHulls(G.shapes);
  cout << "Loaded scene: " << scene << endl;

  MotionProblem P(G);
  P.loadTransitionParameters();


  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTaskMap("position", new DefaultTaskMap(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));

  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                          ARRAY(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                          ARRAY(0.,0.,0.), 1e-3);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly,
                          ARRAY(0.,0.,0.), 1e3,
                          ARRAY(0.,0.,0.), 0.);

  if (useOrientation) {
    c = P.addTaskMap("orientation", new DefaultTaskMap(vecTMT,G,"endeff",ors::Vector(0., 0., 0.)));
    P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                            ARRAY(0.,0.,1.), 1e4,
                            ARRAY(0.,0.,0.), 1e-3);
  }

  if (useCollAvoid) {
    c = P.addTaskMap("collision", new DefaultTaskMap(collTMT, 0, ors::Vector(0., 0., 0.), 0, ors::Vector(0., 0., 0.), ARR(.1)));
    P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e0);
  }

  //-- create the Optimization problem (of type kOrderMarkov)
  P.x0 = ARRAY(0.,0.,0.,0.,-0.2,-.2,0.);

  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt = P.tau;


  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<G.getJointStateDimension()<<endl;

  //-- mini evaluation test:
  arr x(T+1,n);
  x.setZero();

  //-- optimize
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));

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
    G.kinematicsPos(kinPos, NoArr, P.world.getBodyByName("endeff")->index);
    G.kinematicsVec(kinVec, NoArr, P.world.getBodyByName("endeff")->index);
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
  MObject goalMO(&G, MT::String("goal"), MObject::GOAL , 0., ARRAY(0.,0.,1.));
  std::vector<MObject*> mobjects;

  //    mobjects.push_back(new MObject(&G, MT::String("obstacle1"), MObject::OBSTACLE , 0.003, ARRAY(0.,0.,-1.)));
  //    gl.add(drawEnv,mobjects.at(0));


  arr q, dq, q0, dq0;

  // Set start state
  q0 = x[0]; dq0 = 0.*q0;
  G.setJointState(q0,dq0);
  G.calcBodyFramesFromJoints();
  G.getJointState(q);

  //------------------------------------------------//
  // Create pfc from optimal trajectory in task space
  //------------------------------------------------//
  arr x0 = xRef[0];
  double fPos_deviation = 1e-2;
  double fVec_deviation = 1e-3;
  double yCol_deviation = 3e-1;
  double w_reg = 100.;

  Pfc *pfc = new Pfc(G, xRef,2.,x0, q0, goalMO, useOrientation, useCollAvoid,fPos_deviation,fVec_deviation,yCol_deviation,w_reg);
  pfc->scene = scene;
  G.gl().add(drawActTraj,&(pfc->traj));
  G.gl().add(drawRefTraj,&(pfc->trajRef->points));
  G.gl().add(drawPlanTraj,&(pfc->trajWrap->points));

  //------------------------------------------------//
  // Simulate controller
  //------------------------------------------------//
  uint t = 0;
  while((pfc->s.last()<0.99) && t++ < 2*T)
  {
    // move obstacles
    for (std::vector<MObject*>::iterator moIter = mobjects.begin() ; moIter != mobjects.end() ; ++moIter) {
      (*moIter)->move();
    }

    // move goal
    goalMO.move();

    P.world.computeProxies();
    pfc->computeIK(q,dq);

    // sets joint angles AND computes all frames AND update display
    G.setJointState(q);
    G.calcBodyFramesFromJoints();

    G.gl().update();
  }

  //------------------------------------------------//
  // Plot results
  //------------------------------------------------//
  pfc->plotState();

  G.gl().watch();

}



int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

//  runPFC(String("model.kvg"),true,true);

  runPFC(String("scenes/scene1"),true,true);
  runPFC(String("scenes/scene2"),true,true);
  runPFC(String("scenes/scene3"),true,true);
  runPFC(String("scenes/scene4"),true,true);
  runPFC(String("scenes/scene5"),true,true);
  runPFC(String("scenes/scene6"),true,true);
  runPFC(String("scenes/scene7"),true,true);
  runPFC(String("scenes/scene8"),true,true);
  runPFC(String("scenes/scene9"),true,true);

  return 0;
}
