#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include "../splines/spline.h"
#include <GL/glu.h>
#include <Gui/opengl.h>

#include "../pfc/pfc.h"
#include "../pfc/mobject.h"
#include "../mpc/mpc.h"
#include "../dmp/dmp.h"


void drawEnv(void* classP){
  MObject *mo=(MObject*)classP;
  glColor(5);
  glLineWidth(1);
  glBegin(GL_LINES);
  glVertex3f(mo->position(0),mo->position(1),mo->position(2));
  glEnd();
  glLineWidth(1);
}

void drawPoint(void* classP){
  arr *p = (arr*)classP;
  glPointSize(7.0f);
  glLineWidth(2);
  glBegin(GL_POINTS);
  //  cout << p->elem(0) << p->elem(1) << p->elem(2) << endl;
  glVertex3f(p->elem(0), p->elem(1), p->elem(2));
  glVertex3f(p->elem(0), p->elem(1), p->elem(2));
  glEnd();
  glLineWidth(1);
  glPointSize(1.0f);
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
  drawTraj(2,*p,2);
}

void executeTrajectoryPFC(MT::String scene){
  ors::KinematicWorld world(scene);
  arr q, qdot;
  world.getJointState(q, qdot);
  /*
  ** Plan Trajectory
  */
  makeConvexHulls(world.shapes);
  MotionProblem P(world);
  P.loadTransitionParameters();

  arr goal = ARRAY(P.world.getBodyByName("goalRef")->X.pos);

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTaskMap("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, goal, 1e4);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  c = P.addTaskMap("orientation", new DefaultTaskMap(vecTMT,world,"endeff",ors::Vector(0., 0., 1.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(1.,0.,0.), 1e4);
  P.setInterpolatingVelCosts(c,MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  //  c = P.addTaskMap("contact", new DefaultTaskMap(collTMT,-1,NoVector,-1,NoVector,ARR(0.15)));
  //  P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e0);


  //-- create the Optimization problem (of type kOrderMarkov)
  P.x0 = 0.1;

  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt = P.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n); x.setZero();
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
  //  P.costReport();
  //  displayTrajectory(x, 1, world, "planned trajectory", 0.01);

  //-- Transform trajectory into task space
  arr kinPos, kinVec, xRefPos, xRefVec, xRef;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,P.world.getBodyByName("endeff")->index);
    world.kinematicsVec(kinVec,NoArr,P.world.getBodyByName("endeff")->index);
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  xRef = ~cat(~xRefPos,~xRefVec);

  arr knots = linspace(0.,T*dt,xRefPos.d0-1);
  Spline trajPos(knots,xRefPos,2);
  Spline trajVec(knots,xRefVec,2);

  /*
  ** Execute Trajectory
  */
  arr q0 = x[0];
  arr x0 = xRef[0];
  q = P.x0;
  world.setJointState(q,qdot);

  double tau_plan = 0.01;
  double tau_control = 0.001;
  double t = 0.;
  double t_final = T*dt;

  FeedbackMotionControl MP(world, false);
  MP.nullSpacePD.prec=0.;
  double regularization=1e-3;
  PDtask *taskPos = MP.addPDTask("pos", tau_plan*5, 1, posTMT, "endeff");
  PDtask *taskVec = MP.addPDTask("vec", tau_plan*5, 1, vecTMT, "endeff",ARR(0.,0.,1.));
  PDtask *taskCol = MP.addPDTask("col", .02, 1., collTMT, NULL, NoVector, NULL, NoVector, ARR(0.1));
  PDtask *taskHome = MP.addPDTask("home", .02, 0.5, qItselfTMT);

  taskPos->setGains(1.,100.); taskPos->prec=1e0;
  taskVec->setGains(1.,100.); taskVec->prec=1e0;
  taskHome->setGains(0.,10.); taskHome->prec=1e-1;
  taskCol->prec=1e0;


  MObject goalMO(&world, MT::String("goal"), MObject::GOAL , 0.001, ARRAY(0.,0.,1.));
  Pfc* pfc = new Pfc(world,xRef,tau_plan,t_final,x0,q0,goalMO,true);
//  pfc->dsRef = tau_plan/t_final;
//  pfc->dt = tau_plan;

  // gl visualization
  world.gl().add(drawPoint,&(taskPos->y_ref));
  world.gl().add(drawActTraj,&(pfc->traj));
  world.gl().add(drawPlanTraj,&(pfc->trajWrap->points));

  arr state,stateVec;
  // RUN //
  while (pfc->s.last() <0.99){
    if ( (fmod(t,tau_plan-1e-12) < tau_control) ) {
      // Outer Planning Loop [1/tau_plan Hz]
      // Get current task state
      world.kinematicsPos(state,NoArr,P.world.getBodyByName("endeff")->index);
      world.kinematicsVec(stateVec,NoArr,P.world.getBodyByName("endeff")->index);
      state.append(stateVec);

      if (pfc->s.last() < 0.9){
        pfc->goalMO->move();
      }
      pfc->iterate(state);

      arr yNext, ydNext;
      pfc->getNextState(yNext,ydNext);

      // compute desired state postion and velocity
//      arr yNext = pfc->traj[pfc->traj.d0-1];
//      arr ydNext = pfc->dsRef*pfc->trajWrap->deval(pfc->s.last())/tau_plan;
//      arr dir = pfc->trajWrap->deval(pfc->s.last());
//      dir = dir/length(dir);
//      arr ydNext = dir*pfc->dsRef*length(pfc->trajRef->deval(pfc->s.last()))/tau_plan;
      taskPos->y_ref = yNext.subRange(0,2);
      taskPos->v_ref = ydNext.subRange(0,2);
      taskVec->y_ref = yNext.subRange(3,5);
      taskVec->v_ref = ydNext.subRange(3,5);
      world.watch(false, STRING(t));
    }

    // Inner Controlling Loop [1/tau_control Hz]
    MP.setState(q, qdot);
    //    world.stepPhysx(tau_control);
    world.computeProxies();

    arr a = MP.operationalSpaceControl(regularization);
    q += tau_control*qdot;
    qdot += tau_control*a;
    t += tau_control;
  }
  pfc->plotState();
  world.watch(true,STRING(t));
}



void executeTrajectoryMPC(MT::String scene){
  ors::KinematicWorld world(scene);
  arr q, qdot;
  world.getJointState(q, qdot);
  /*
  ** Plan Trajectory
  */
  makeConvexHulls(world.shapes);
  MotionProblem P(world);

  P.loadTransitionParameters();

  arr goalRef = ARRAY(P.world.getBodyByName("goalRef")->X.pos);

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTaskMap("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, goalRef, 1e4);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  c = P.addTaskMap("orientation", new DefaultTaskMap(vecTMT,world,"endeff",ors::Vector(0., 0., 1.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(1.,0.,0.), 1e4);
  P.setInterpolatingVelCosts(c,MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  P.x0 = 0.1;

  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt = P.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n);
  x.setZero();
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
  //  P.costReport();
  //  displayTrajectory(x, 1, world, "planned trajectory", 0.01);


  /*
  ** Execute Trajectory
  */
  arr q0 = x[0];
  q = P.x0;
  world.setJointState(q,0.*qdot);

  double tau_plan = P.tau;
  double tau_control = 0.01;
  double t = 0.;
  double t_final = T*dt;

  FeedbackMotionControl MP(world, false);
  PDtask *jointPos = MP.addPDTask("pos", tau_plan*5, 1, qItselfTMT, "endeff");

  MObject goalMO(&world, MT::String("goal"), MObject::GOAL , 0.001, ARRAY(0.,0.,1.));

  MPC mpc(P,x);
  // gl visualization
  world.gl().add(drawActTraj,&(mpc.x_cart));

  // RUN //
  while ((world.getBodyByName("endeff")->X.pos - goalMO.position).length() >1e-2) {
    if ( (fmod(t,tau_plan-1e-12) < tau_control) ) {
      goalMO.move();
      mpc.replan(goalMO.position,q);
      jointPos->y_ref = mpc.x[1];
      jointPos->v_ref = (mpc.x[2]-mpc.x[1])/tau_plan;
    }


    // Inner Controlling Loop [1/tau_control Hz]
    arr qw;world.getJointState(qw);
    MP.setState(q, qdot);
    //    world.stepPhysx(tau_control);
    world.computeProxies();

    arr a = MP.operationalSpaceControl();
    q += tau_control*qdot;
    qdot += tau_control*a;
    t += tau_control;
    world.watch(false, STRING(t));
  }
  world.watch(true,STRING(t));
}


void executeTrajectoryDMP(MT::String scene){
  ors::KinematicWorld world(scene);
  arr q, qdot;
  world.getJointState(q, qdot);

  /*********************************
  ** Plan Trajectory ***************
  *********************************/
  makeConvexHulls(world.shapes);
  MotionProblem P(world);

  P.loadTransitionParameters();

  TaskCost *c;
  arr goalRef = ARRAY(P.world.getBodyByName("goalRef")->X.pos);
  c = P.addTaskMap("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, goalRef, 1e4);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  c = P.addTaskMap("orientation", new DefaultTaskMap(vecTMT,world,"endeff",ors::Vector(0., 0., 1.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(1.,0.,0.), 1e4);
  P.setInterpolatingVelCosts(c,MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  P.x0 = 0.1;

  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt = P.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n);
  x.setZero();
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
  //  P.costReport();
  //  displayTrajectory(x, 1, world, world.gl(),"planned trajectory", 0.01);


  //-- Transform trajectory into task space
  arr kinPos, kinVec, xRefPos, xRefVec, xRef;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,P.world.getBodyByName("endeff")->index);
    world.kinematicsVec(kinVec,NoArr,P.world.getBodyByName("endeff")->index);
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  xRef = ~cat(~xRefPos,~xRefVec);;

  /*********************************
  ** Execute Trajectory ************
  *********************************/
  arr q0 = x[0];
  q = P.x0;
  world.setJointState(q,0.*qdot);

  double tau_plan = P.tau;
  double tau_control = 0.01;
  double t = 0.;
  double t_final = T*dt;

  FeedbackMotionControl MP(world);
  PDtask *taskPos = MP.addPDTask("pos", tau_plan*5, 1, posTMT, "endeff");
  PDtask *taskVec = MP.addPDTask("vec", tau_plan*5, 1, vecTMT, "endeff",ARR(0.,0.,1.));

  MObject goalMO(&world, MT::String("goal"), MObject::GOAL , 0.001, ARRAY(0.,0.,1.));

  DMP dmp(xRef,100,tau_plan);
  dmp.trainDMP();

  // gl visualization
  world.gl().add(drawPoint,&(taskPos->y_ref));
  world.gl().add(drawActTraj,&(dmp.y_ref));
  world.gl().add(drawPlanTraj,&(dmp.y_bk));

  // RUN //
  while ((world.getBodyByName("endeff")->X.pos - goalMO.position).length() >1e-2) {
    if ( (fmod(t,tau_plan-1e-12) < tau_control) ) {
      //      goalMO.move();

      dmp.goal.subRange(0,2) = goalMO.position;
      dmp.iterate();

      arr state, dstate;
      state = dmp.Y;
      dstate = dmp.Yd;

      taskPos->y_ref = state.subRange(0,2);
      taskPos->v_ref = dstate.subRange(0,2);
      taskVec->y_ref = state.subRange(3,5);
      taskVec->v_ref = dstate.subRange(3,5);
    }


    // Inner Controlling Loop [1/tau_control Hz]
    MP.setState(q, qdot);
    // world.stepPhysx(tau_control);
    world.computeProxies();

    arr a = MP.operationalSpaceControl();
    q += tau_control*qdot;
    qdot += tau_control*a;
    t += tau_control;
    world.watch(false, STRING(t));
  }
  dmp.plotDMP();
  world.watch(true,STRING(t));
}

int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);
  executeTrajectoryPFC(String("scene1.ors"));
  executeTrajectoryMPC(String("scene1.ors"));
  executeTrajectoryDMP(String("scene1.ors"));
  return 0;
}
