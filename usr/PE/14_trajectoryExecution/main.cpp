#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include "../splines/spline.h"
#include <GL/glu.h>
#include <Gui/opengl.h>
#include <iomanip>

#include "../pfc/pfc.h"
#include "../pfc/mobject.h"
#include "../mpc/mpc.h"
#include "../dmp/dmp.h"

#define VISUALIZE 0

// GLOBAL OPTION VARS
double goalAccuracy;
String evalName;
String sceneName;
int numScenes;
enum ControlType {CT_DMP, CT_PFC, CT_MPC};
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


void executeTrajectory(String scene, ControlType cType){
  //-----------------------------------------------------//
  //--------------------- optimize reference trajectory //
  //-------------------------------------------------- //
  String currScene = STRING("scenes/"<<evalName<<"/"<<scene);
  String folder = STRING("out/"<<evalName<<"/"<<scene);
  cout << currScene << endl;

  ors::KinematicWorld world(currScene);
#if VISUALIZE
  world.gl().resize(800, 800);
#endif
  arr q, qdot;
  world.getJointState(q, qdot);

  // Plan Trajectory
  makeConvexHulls(world.shapes);
  MotionProblem P(world);
  P.loadTransitionParameters();

  arr goal = ARRAY(P.world.getBodyByName("goalRef")->X.pos);

  //-- create an optimal trajectory to trainTarget
  TaskCost *c;
  c = P.addTask("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, goal, 1e4);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  c = P.addTask("orientation", new DefaultTaskMap(vecTMT,world,"endeff",ors::Vector(0., 0., 1.)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(1.,0.,0.), 1e4);
  P.setInterpolatingVelCosts(c,MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

//  c = P.addTask("contact", new DefaultTaskMap(collTMT,-1,NoVector,-1,NoVector,ARR(0.1)));
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
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));
  //  P.costReport();
  //  displayTrajectory(x, 1, world, "planned trajectory", 0.01);

  //-- Transform trajectory into task space
  arr kinPos, kinVec, xRefPos, xRefVec, xRef;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,P.world.getBodyByName("endeff"));
    world.kinematicsVec(kinVec,NoArr,P.world.getBodyByName("endeff"));
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  xRef = ~cat(~xRefPos,~xRefVec);

  // set initial positions
  arr q0 = x[0];
  arr x0 = xRef[0];
  q = P.x0;
  world.setJointState(q,qdot);


  //-----------------------------------------------------//
  //-------------------------------- execute controller //
  //-------------------------------------------------- //
  double tau_plan = 0.01;
  double tau_control = 0.001;
  double t = 0.;
  double t_final = T*dt;

  arr dir;
  if (moveGoal){
    dir = ARRAY(world.getBodyByName("dir")->X.pos);
    cout << dir << endl;
  } else {
    dir = ARRAY(1.,0.,0.);
  }

  std::vector<MObject*> mobstacles;
  if (moveObs) {
    arr obsdir = ARRAY(world.getBodyByName("obsdir")->X.pos);
    mobstacles.push_back(new MObject(&world, MT::String("obstacle"), MObject::OBSTACLE , 0.0005, obsdir));
  }


  MObject goalMO(&world, MT::String("goal"), MObject::GOAL , 0.0005, dir);

  FeedbackMotionControl MP(world, false);
  PDtask *taskPos, *taskVec, *taskHome, *taskCol, *jointPos;
  double regularization = 1e-3;

  // initialize controllers
  Pfc* pfc; MPC* mpc; DMP* dmp;
  switch (cType) {
  case(CT_DMP):
    MP.nullSpacePD.prec=0.;
    taskPos = MP.addPDTask("pos", tau_plan*5, 1, posTMT, "endeff");
    taskVec = MP.addPDTask("vec", tau_plan*5, 1, vecTMT, "endeff",ARR(0.,0.,1.));
    taskCol = MP.addPDTask("col", .02, 1., collTMT, NULL, NoVector, NULL, NoVector, ARR(0.1));
    taskHome = MP.addPDTask("home", .02, 0.5, new TaskMap_qItself());
    taskHome->setGains(0.,10.); taskHome->prec=1e-1;
    taskCol->prec=1e0;
    taskPos->prec=1e0;
    taskVec->prec=1e0;
    dmp = new DMP(xRef,100,tau_plan);
    dmp->trainDMP();
    // world.gl().add(drawPoint,&(taskPos->y_ref));
    // world.gl().add(drawActTraj,&(dmp->y_ref));
    // world.gl().add(drawPlanTraj,&(dmp->y_bk));
    break;
  case(CT_MPC):
    MP.nullSpacePD.prec=0.;
    jointPos = MP.addPDTask("pos", tau_plan*2, 1, new TaskMap_qItself());
    jointPos->setGains(100.,100.);
    mpc = new MPC(P,x);
    // world.gl().add(drawActTraj,&(mpc->x_cart));
    break;
  case(CT_PFC):
    MP.nullSpacePD.prec=0.;
    taskPos = MP.addPDTask("pos", tau_plan*5, 1, posTMT, "endeff");
    taskVec = MP.addPDTask("vec", tau_plan*5, 1, vecTMT, "endeff",ARR(0.,0.,1.));
    taskCol = MP.addPDTask("col", .02, 1., collTMT, NULL, NoVector, NULL, NoVector, ARR(0.1));
    taskHome = MP.addPDTask("home", .02, 0.5, new TaskMap_qItself());

    taskPos->setGains(1.,100.); taskPos->prec=1e0;
    taskVec->setGains(1.,100.); taskVec->prec=1e0;
    taskHome->setGains(0.,10.); taskHome->prec=1e-1;
    taskCol->prec=1e0;
    pfc = new Pfc(world,xRef,tau_plan,t_final,x0,q0,goalMO,true);
    // world.gl().add(drawPoint,&(taskPos->y_ref));
    // world.gl().add(drawActTraj,&(pfc->traj));
    // world.gl().add(drawPlanTraj,&(pfc->trajWrap->points));
    break;
  }

  // init bookkeeping
  q_bk = ~q;
  x_bk = ~x0;
  goal_bk = ~goalMO.position;
  ct_bk = ARR(0);


  world.setJointState(q,qdot); 
  arr state,stateVec;
  // RUN //
  while (((world.getBodyByName("endeff")->X.pos - goalMO.position).length() > goalAccuracy) && t < maxDuration && sum(ct_bk)<50.) {
    if ( (fmod(t,tau_plan-1e-12) < tau_control) ) {
      // Outer Planning Loop [1/tau_plan Hz]
      MT::timerStart(true);
      // Get current task state
      world.kinematicsPos(state,NoArr,P.world.getBodyByName("endeff"));
      world.kinematicsVec(stateVec,NoArr,P.world.getBodyByName("endeff"));
      state.append(stateVec);

      // Move goal
      if (t < 0.7*t_final && moveGoal) {
        goalMO.move();
      }
      if (moveObs) {
        mobstacles[0]->move();
      }


      arr yNext, ydNext;
      switch (cType) {
      case(CT_DMP):
        dmp->goal.subRange(0,2) = goalMO.position;
        dmp->iterate();

        yNext = dmp->Y;
        ydNext = dmp->Yd;
        break;
      case(CT_MPC):
#if VISUALIZE
        world.watch(false, STRING(t));
#endif
        if (mpc->P.T<2) {
          t = maxDuration;
          break;
        }
        mpc->replan(goalMO.position,q);
        jointPos->y_ref = mpc->x[1];
        jointPos->v_ref = (mpc->x[1]-mpc->x[0])/tau_plan;
        break;
      case(CT_PFC):
        pfc->iterate(state);
        pfc->getNextState(yNext,ydNext);
        break;
      }

      if (cType == CT_DMP || cType == CT_PFC) {
        taskPos->y_ref = yNext.subRange(0,2);
        taskPos->v_ref = ydNext.subRange(0,2);
        taskVec->y_ref = yNext.subRange(3,5);
        taskVec->v_ref = ydNext.subRange(3,5);
#if VISUALIZE
        world.watch(false, STRING(t));
#endif
      }
      ct_bk.append(MT::timerRead());
    }

    // Inner Controlling Loop [1/tau_control Hz]
    MP.setState(q, qdot);

    // world.stepPhysx(tau_control);
    world.stepSwift();

    arr qddot = MP.operationalSpaceControl();//MP.operationalSpaceControl(regularization);
    q += tau_control*qdot;
    qdot += tau_control*qddot;
    t += tau_control;

    // Bookkeeping
    q_bk.append(q);
    x_bk.append(state);
    goal_bk.append(goalMO.position);
  }
  world.watch(true,STRING(t));


  switch (cType) {
  case(CT_DMP):
    folder = STRING(folder<<"_DMP_");
    break;
  case(CT_MPC):
    folder = STRING(folder<<"_MPC_");
    break;
  case(CT_PFC):
    folder = STRING(folder<<"_PFC_");
    break;
  }

  // save bookkeeping files
  write(LIST<arr>(q_bk),STRING(folder<<"q_bk.output"));
  write(LIST<arr>(x_bk),STRING(folder<<"x_bk.output"));
  write(LIST<arr>(goal_bk),STRING(folder<<"goal_bk.output"));
  write(LIST<arr>(ct_bk),STRING(folder<<"ct_bk.output"));

  write(LIST<arr>(xRef),STRING(folder<<"xRef.output"));
  write(ARR(tau_control),STRING(folder<<"tau_control.output"));
  write(ARR(tau_plan),STRING(folder<<"tau_plan.output"));
  write(ARR(numScenes),STRING("out/"<<evalName<<"/"<<"numScenes.output"));

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

  //-------------// Run Evaluation with all methods for each scene //-------------//
  int sceneIter = 1;
  while (sceneIter <= numScenes) {
    String currScene = STRING(sceneName<<sceneIter);
    cout << sceneIter << endl;
    executeTrajectory(currScene,CT_MPC);
    q_bk.clear(); x_bk.clear(); goal_bk.clear(); ct_bk.clear();
    executeTrajectory(currScene,CT_DMP);
    q_bk.clear(); x_bk.clear(); goal_bk.clear(); ct_bk.clear();
    executeTrajectory(currScene,CT_PFC);
    q_bk.clear(); x_bk.clear(); goal_bk.clear(); ct_bk.clear();
    sceneIter++;
  }

  return 0;
}


