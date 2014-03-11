#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <GL/glu.h>
#include <Gui/opengl.h>
#include <iomanip>
#include <Algo/spline.h>
#include <Motion/mobject.h>
#include <Motion/adaptiveMotionExecution.h>

#define VISUALIZE 1

// GLOBAL OPTION VARS
double goalAccuracy;
String evalName;
String sceneName;
int numScenes;
double maxDuration;
bool moveGoal;
bool moveObs;

// BOOKKEEPING VARS
arr q_bk;    // joint angles at each time step
arr x_bk;    // task variables at each time step
arr goal_bk; // goal position at each time step
arr ct_bk;   // computational time at each time step


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
  glVertex3f(p->elem(0), p->elem(1), p->elem(2));
  glVertex3f(p->elem(0), p->elem(1), p->elem(2));
  glEnd();
  glLineWidth(1);
  glPointSize(1.0f);
}

void drawVector(uint color, arr& p){
  //  arr *p = (arr*)classP;
  glPointSize(7.0f);
  glColor(color);
  glLineWidth(2);
  glBegin(GL_LINES);
  glVertex3f(p(0), p(1), p(2));
  glVertex3f(p(0)+p(3)*0.1, p(1)+p(4)*0.1, p(2)+p(5)*0.1);
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

void drawCurrentDir(void* classP){
  arr *p = (arr*)classP;
  drawVector(5,*p);
}
void drawDesiredDir(void* classP){
  arr *p = (arr*)classP;
  drawVector(2,*p);
}

void drawActTraj(void* classP){
  arr *p = (arr*)classP;
  drawTraj(5,*p,1);
}

void drawPlanTraj(void* classP){
  arr *p = (arr*)classP;
  drawTraj(2,*p,2);
}


void executeTrajectoryWholeBody(String scene){
  //-----------------------------------------------------//
  //--------------------- optimize reference trajectory //
  //-------------------------------------------------- //
  String folder = STRING("out/"<<scene);
  cout << scene << endl;

  ors::KinematicWorld world(scene);
  cout << world.getJointStateDimension() << endl;
#if VISUALIZE
  world.gl().resize(800, 800);
#endif
  arr q, qdot;
  world.getJointState(q, qdot);

  // Plan Trajectory
  makeConvexHulls(world.shapes);
  MotionProblem P(world);
  P.loadTransitionParameters();

  arr Rgoal = ARRAY(P.world.getBodyByName("RgoalRef")->X.pos);
  arr Lgoal = ARRAY(P.world.getBodyByName("LgoalRef")->X.pos);

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTaskMap("position_right_hand", new DefaultTaskMap(posTMT,world,"endeffR", ors::Vector(0., 0., 0.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, Rgoal, 1e5);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  c = P.addTaskMap("position_left_hand", new DefaultTaskMap(posTMT,world,"endeffL", ors::Vector(0., 0., 0.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, Lgoal, 1e5);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  //  c = P.addTaskMap("orientation", new DefaultTaskMap(vecTMT,world,"endeff",ors::Vector(0., 0., 1.)));
  //  P.setInterpolatingCosts(c, MEotionProblem::finalOnly, ARRAY(-0.5,0.3,0.8), 1e3);
  //  P.setInterpolatingVelCosts(c,MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  c = P.addTaskMap("qLimits", new DefaultTaskMap(qLimitsTMT,world));
  P.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e0);
  P.setInterpolatingVelCosts(c,MotionProblem::constant,ARRAY(0.),1e1);

  c = P.addTaskMap("homing", new DefaultTaskMap(qItselfTMT,world));
  P.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),0);
  P.setInterpolatingVelCosts(c,MotionProblem::constant,ARRAY(0.),1e0);


  //-- create the Optimization problem (of type kOrderMarkov)
  P.x0 = {0.,0.,0.,0.,0.,0.,0.,0.,-0.3,-0.3,0.,0.,0.,0.,0.,-0.3,-0.3,0};

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
  arr kinPos, kinVec, xRefPosR, xRefVecR, xRefPosL, xRefVecL, xRefR, xRefL;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,P.world.getBodyByName("endeffR")->index);
    world.kinematicsVec(kinVec,NoArr,P.world.getBodyByName("endeffR")->index);
    xRefPosR.append(~kinPos);
    xRefVecR.append(~kinVec);

    world.kinematicsPos(kinPos,NoArr,P.world.getBodyByName("endeffL")->index);
    world.kinematicsVec(kinVec,NoArr,P.world.getBodyByName("endeffL")->index);
    xRefPosL.append(~kinPos);
    xRefVecL.append(~kinVec);
  }

  xRefR = ~cat(~xRefPosR,~xRefVecR);
  xRefL = ~cat(~xRefPosL,~xRefVecL);

  // set initial positions
  arr q0 = x[0];
  arr x0_L = xRefL[0];
  arr x0_R = xRefR[0];
  q = P.x0;
  world.setJointState(q,qdot);


  //-----------------------------------------------------//
  //-------------------------------- execute controller //
  //-------------------------------------------------- //
  double tau_plan = 0.01;
  double tau_control = 0.001;
  double t = 0.;
  double t_final = T*dt;

  arr dirR = ARRAY(0.,0.,-1.);
  arr dirL = ARRAY(0.,0.,0.);

  MObject goalMO_R(&world, MT::String("Rgoal"), MObject::GOAL , 0.0005, dirL);
  MObject goalMO_L(&world, MT::String("Lgoal"), MObject::GOAL , 0.0005, dirR);

  FeedbackMotionControl MP(world, false);
  PDtask *taskPosR, *taskVecR, *taskHome, *taskCol, *taskLimits;
  PDtask *taskPosL, *taskVecL;
  double regularization = 1e-2;

  // initialize controllers
  AdaptiveMotionExecution* amexL;
  AdaptiveMotionExecution* amexR;

  //  MP.nullSpacePD.prec=0.;
  MP.nullSpacePD.active=false;
  taskPosR = MP.addPDTask("posR", tau_plan*5, 1, posTMT, "endeffR");
  taskVecR = MP.addPDTask("vecR", tau_plan*5, 1, vecTMT, "endeffR",ARR(0.,0.,1.));
  taskPosL = MP.addPDTask("posL", tau_plan*5, 1, posTMT, "endeffL");
  taskVecL = MP.addPDTask("vecL", tau_plan*5, 1, vecTMT, "endeffL",ARR(0.,0.,1.));
  taskHome = MP.addPDTask("home", .02, 0.5, qItselfTMT);
  taskLimits = MP.addPDTask("limits", .02, 0.5, qLimitsTMT);

  //  taskPos->setGains(10.,100.); taskPos->prec=1e1;
  //  taskVec->setGains(10.,100.); taskVec->prec=1e0;
  //  taskHome->setGains(0.,100.); taskHome->prec=1e-1;
  //  taskLimits->setGains(0.,100.); taskLimits->prec=1e2;

  taskPosL->setGains(1.,100.); taskPosL->prec=1e5;
  taskVecL->setGains(1.,100.); taskVecL->prec=1e2;
  taskPosR->setGains(1.,100.); taskPosR->prec=1e5;
  taskVecR->setGains(1.,100.); taskVecR->prec=1e2;
  taskHome->setGains(1.,100.); taskHome->prec=1e2;
  taskLimits->setGains(100.,100.); taskLimits->prec=1e3;


  amexL = new AdaptiveMotionExecution(world,xRefL,tau_plan,t_final,x0_L,q0,goalMO_L,true);
  amexR = new AdaptiveMotionExecution(world,xRefR,tau_plan,t_final,x0_R,q0,goalMO_R,true);

  world.gl().add(drawPoint,&(taskPosL->y_ref));
  world.gl().add(drawPoint,&(taskPosR->y_ref));
  world.gl().add(drawActTraj,&(amexL->traj));
  world.gl().add(drawPlanTraj,&(amexL->trajRef->points));
  world.gl().add(drawActTraj,&(amexR->traj));
  world.gl().add(drawPlanTraj,&(amexR->trajRef->points));
  arr current_dir = ARRAY(1.,1.,1.,0.,0.,0.);
  world.gl().add(drawCurrentDir,&current_dir);
  arr des_dir = current_dir;
  world.gl().add(drawDesiredDir,&(des_dir));


  // init bookkeeping
  q_bk = ~q;
  x_bk = ~x0_R;
  goal_bk = ~goalMO_R.position;
  ct_bk = ARR(0);

  world.setJointState(q,qdot);
  arr state,stateVec;
  // RUN //
  while (((world.getShapeByName("endeffR")->X.pos - goalMO_R.position).length() > goalAccuracy) && t < maxDuration && sum(ct_bk)<50.) {
    if ( (fmod(t,tau_plan-1e-12) < tau_control) ) {
      // Outer Planning Loop [1/tau_plan Hz]
      MT::timerStart(true);
      // Get current task state
      world.kinematicsPos(state,NoArr,P.world.getBodyByName("endeffL")->index);
      world.kinematicsVec(stateVec,NoArr,P.world.getBodyByName("endeffL")->index);
      state.append(stateVec);
      // Move goal
      if (t < 0.7*t_final && moveGoal) {
        goalMO_R.move();
        goalMO_L.move();
      }

      arr yNext, ydNext;
      amexL->iterate(state);
      amexL->getNextState(yNext,ydNext);
      taskPosL->y_ref = yNext.subRange(0,2);
      taskPosL->v_ref = ydNext.subRange(0,2);
      taskVecL->y_ref = yNext.subRange(3,5);
      taskVecL->v_ref = ydNext.subRange(3,5);

      world.kinematicsPos(state,NoArr,P.world.getBodyByName("endeffR")->index);
      world.kinematicsVec(stateVec,NoArr,P.world.getBodyByName("endeffR")->index);
      state.append(stateVec);

      amexR->iterate(state);
      amexR->getNextState(yNext,ydNext);
      taskPosR->y_ref = yNext.subRange(0,2);
      taskPosR->v_ref = ydNext.subRange(0,2);
      taskVecR->y_ref = yNext.subRange(3,5);
      taskVecR->v_ref = ydNext.subRange(3,5);

#if VISUALIZE
      current_dir = state;
      des_dir = yNext;
      world.watch(false, STRING(t));
#endif
      ct_bk.append(MT::timerRead());
    }

    // Inner Controlling Loop [1/tau_control Hz]
    MP.setState(q, qdot);

    // world.stepPhysx(tau_control);
    world.computeProxies();

    arr qddot = MP.operationalSpaceControl();//MP.operationalSpaceControl(regularization);
    q += tau_control*qdot;
    qdot += tau_control*qddot;
    t += tau_control;

    // Bookkeeping
    q_bk.append(q);
    x_bk.append(state);
    goal_bk.append(goalMO_R.position);
  }
  world.watch(true,STRING(t));

  // save bookkeeping files
  write(LIST<arr>(q_bk),STRING(folder<<"q_bk.output"));
  write(LIST<arr>(x_bk),STRING(folder<<"x_bk.output"));
  write(LIST<arr>(goal_bk),STRING(folder<<"goal_bk.output"));
  write(LIST<arr>(ct_bk),STRING(folder<<"ct_bk.output"));

  write(LIST<arr>(xRefR),STRING(folder<<"xRef.output"));
  write(ARR(tau_control),STRING(folder<<"tau_control.output"));
  write(ARR(tau_plan),STRING(folder<<"tau_plan.output"));
  write(ARR(numScenes),STRING(folder<<"numScenes.output"));

  return;
}

void executeTrajectoryRightArm(String scene){
  //-----------------------------------------------------//
  //--------------------- optimize reference trajectory //
  //-------------------------------------------------- //
  String folder = STRING("out/"<<scene);
  cout << scene << endl;

  ors::KinematicWorld world(scene);
  cout << world.getJointStateDimension() << endl;
#if VISUALIZE
  world.gl().resize(800, 800);
#endif

  arr q0 = {0.,0.,0.,0.,-0.2,-0.2,0.};
  world.setJointState(q0,0.*q0);

  arr q, qdot;
  world.getJointState(q, qdot);


  // Plan Trajectory
  makeConvexHulls(world.shapes);
  MotionProblem P(world);
  P.loadTransitionParameters();

  arr Rgoal = ARRAY(P.world.getBodyByName("goalRef")->X.pos);

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTaskMap("position_right_hand", new DefaultTaskMap(posTMT,world,"endeffR", ors::Vector(0., 0., 0.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, Rgoal, 1e4);
  //  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  //  c = P.addTaskMap("orientation", new DefaultTaskMap(vecTMT,world,"endeff",ors::Vector(0., 0., 1.)));
  //  P.setInterpolatingCosts(c, MEotionProblem::finalOnly, ARRAY(-0.5,0.3,0.8), 1e3);
  //  P.setInterpolatingVelCosts(c,MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  c = P.addTaskMap("qLimits", new DefaultTaskMap(qLimitsTMT,world));
  P.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e0,ARRAY(0.),1e0);
  P.setInterpolatingVelCosts(c,MotionProblem::constant,ARRAY(0.),1e-1);

  c = P.addTaskMap("homing", new DefaultTaskMap(qItselfTMT,world));
  P.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),0);
  //  P.setInterpolatingVelCosts(c,MotionProblem::constant,ARRAY(0.),1e0);
  P.setInterpolatingVelCosts(c,MotionProblem::finalOnly,ARRAY(0.),1e2);


  //-- create the Optimization problem (of type kOrderMarkov)
  P.x0 = {0.,0.,0.,0.,-0.3,-0.3,0.};

  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  double dt = P.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n); x.setZero();
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
  P.costReport();
//  displayTrajectory(x, 1, world, "planned trajectory", 0.01);

  //-- Transform trajectory into task space
  arr kinPos, kinVec, xRefPosR, xRefVecR, xRefPosL, xRefVecL, xRefR, xRefL;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,P.world.getBodyByName("endeffR")->index);
    world.kinematicsVec(kinVec,NoArr,P.world.getBodyByName("endeffR")->index);
    xRefPosR.append(~kinPos);
    xRefVecR.append(~kinVec);
  }

  xRefR = ~cat(~xRefPosR,~xRefVecR);

  // set initial positions
//  arr q0 = x[0];
  arr x0_R = xRefR[0];
  q = P.x0;
  world.setJointState(q,qdot);


  //-----------------------------------------------------//
  //-------------------------------- execute controller //
  //-------------------------------------------------- //
  double tau_plan = 0.01;
  double tau_control = 0.001;
  double t = 0.;
  double t_final = T*dt;

  arr dirR = ARRAY(0.,0.,-1.);
  arr dirL = ARRAY(0.,0.,0.);

  MObject goalMO(&world, MT::String("goal"), MObject::GOAL , 0.0005, dirL);

  FeedbackMotionControl MP(world, false);
  PDtask *taskPosR, *taskVecR, *taskHome, *taskCol, *taskLimits, *qitself;
  double regularization = 1e-2;

  // initialize controllers
  AdaptiveMotionExecution* amexR;

  //  MP.nullSpacePD.prec=0.;
  MP.nullSpacePD.active=false;
  taskPosR = MP.addPDTask("posR", tau_plan*5, 1, posTMT, "endeffR");
  taskVecR = MP.addPDTask("vecR", tau_plan*5, 1, vecTMT, "endeffR",ARR(0.,0.,1.));
  qitself = MP.addPDTask("qitself", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, 0.01*MP.H_rate_diag);
  cout << MP.H_rate_diag << endl;
  double t_PD = tau_plan/4.;
  double damp_PD = 0.9;
  taskPosR->setGainsAsNatural(t_PD,damp_PD); taskPosR->prec=1e5;
  taskVecR->setGainsAsNatural(t_PD,damp_PD); taskVecR->prec=1e3;
  qitself->setGainsAsNatural(t_PD,damp_PD); qitself->prec=1e2;
  qitself->Pgain = 0.;

  cout << "taskPosR->Pgain " << taskPosR->Pgain << endl;
  cout << "taskPosR->Dgain " << taskPosR->Dgain << endl;
  cout << "taskPosR->prec " << taskPosR->prec << endl;
  cout << "taskVecR->Pgain " << taskVecR->Pgain << endl;
  cout << "taskVecR->Dgain " << taskVecR->Dgain << endl;
  cout << "taskVecR->prec " << taskVecR->prec << endl;
  cout << "qitself->Pgain " << qitself->Pgain << endl;
  cout << "qitself->Dgain " << qitself->Dgain << endl;
  cout << "qitself->prec " << qitself->prec << endl;

  amexR = new AdaptiveMotionExecution(world,xRefR,tau_plan,t_final,x0_R,q0,goalMO,true);

  world.gl().add(drawPoint,&(taskPosR->y_ref));
  world.gl().add(drawActTraj,&(amexR->traj));
  world.gl().add(drawPlanTraj,&(amexR->trajRef->points));
  arr current_dir = ARRAY(1.,1.,1.,0.,0.,0.);
  world.gl().add(drawCurrentDir,&current_dir);
  arr des_dir = current_dir;
  world.gl().add(drawDesiredDir,&(des_dir));


  // init bookkeeping
  q_bk = ~q;
  x_bk = ~x0_R;
  goal_bk = ~goalMO.position;
  ct_bk = ARR(0);

  world.setJointState(q,qdot);
  arr state,stateVec;
  // RUN //
  while (((world.getShapeByName("endeffR")->X.pos - goalMO.position).length() > goalAccuracy) && t < maxDuration*2. && sum(ct_bk)<50.) {
    if ( (fmod(t,tau_plan-1e-12) < tau_control) ) {
      // Outer Planning Loop [1/tau_plan Hz]
      MT::timerStart(true);
      // Get current task state
      world.kinematicsPos(state,NoArr,P.world.getBodyByName("endeffR")->index);
      world.kinematicsVec(stateVec,NoArr,P.world.getBodyByName("endeffR")->index);
      state.append(stateVec);
      // Move goal
      if (t < 0.7*t_final && moveGoal) {
        goalMO.move();
      }

      arr yNext, ydNext;
      amexR->iterate(state);
      amexR->getNextState(yNext,ydNext);
      taskPosR->y_ref = yNext.subRange(0,2);
      taskPosR->v_ref = ydNext.subRange(0,2);
      taskVecR->y_ref = yNext.subRange(3,5);
      taskVecR->v_ref = ydNext.subRange(3,5);

//      taskPosR->y_ref = goalMO.position;
//      taskPosR->v_ref = 0.;//ydNext.subRange(0,2);
//      taskVecR->y_ref = goalMO.orientation;//yNext.subRange(3,5);
//      taskVecR->v_ref = 0.;//ydNext.subRange(3,5);


#if VISUALIZE
      current_dir = state;
      des_dir = yNext;
      world.watch(false, STRING(t));
#endif
      ct_bk.append(MT::timerRead());
    }

    // Inner Controlling Loop [1/tau_control Hz]
    MP.setState(q, qdot);

    // world.stepPhysx(tau_control);
    world.computeProxies();

    arr qddot = MP.operationalSpaceControl();//MP.operationalSpaceControl(regularization);
    q += tau_control*qdot;
    qdot += tau_control*qddot;
    t += tau_control;

    // Bookkeeping
    q_bk.append(q);
    x_bk.append(state);
    goal_bk.append(goalMO.position);
  }

  // save bookkeeping files
  write(LIST<arr>(q_bk),STRING(folder<<"q_bk.output"));
  write(LIST<arr>(x_bk),STRING(folder<<"x_bk.output"));
  write(LIST<arr>(goal_bk),STRING(folder<<"goal_bk.output"));
  write(LIST<arr>(ct_bk),STRING(folder<<"ct_bk.output"));

  write(LIST<arr>(xRefR),STRING(folder<<"xRef.output"));
  write(ARR(tau_control),STRING(folder<<"tau_control.output"));
  write(ARR(tau_plan),STRING(folder<<"tau_plan.output"));
  write(ARR(numScenes),STRING(folder<<"numScenes.output"));

  amexR->plotState();
  world.watch(true,STRING(t));

  return;
}


int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);
  //-------------// Init Evaluation Parameters //-------------//
  // distance between robot and endeffector when movement is stopped
  goalAccuracy = MT::getParameter<double>("goalAccuracy");
  // folder name of evaluation result
  evalName = MT::getParameter<String>("evalName");
  //--- scene name describing robot and environment
  // the scene names is assembled by sceneName[1-numScenes]
  sceneName = MT::getParameter<String>("sceneName");
  numScenes = MT::getParameter<int>("numScenes");
  // time after which motion is stopped
  maxDuration = MT::getParameter<double>("maxDuration");
  // flag if goal should move
  moveGoal = MT::getParameter<int>("moveGoal");
  // flag if obs should move
  moveObs = MT::getParameter<int>("moveObstacle");


  // Whole Body
  String currScene = STRING("whole_body_scene");
  //  executeTrajectoryWholeBody(currScene);

  // Right Arm
  currScene = STRING("right_arm_scene");
  executeTrajectoryRightArm(currScene);

  return 0;
}

