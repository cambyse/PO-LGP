#include "roopi.h"

#include <Core/module.h>
#include <Algo/spline.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <Motion/motion.h>
//#include <Actions/RelationalMachineModule.h>
//#include <Actions/ActivitySpinnerModule.h>
#include <RosCom/serviceRAP.h>
#include <RosCom/baxter.h>

#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/subscribeTabletop.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

#include <Roopi/loggingModule.h>


#define baxter 0

//==============================================================================
/*

(mt) The mid-term concept should be: Roopi allows you to create (and return) little
handles on 'tasks' or 'things'. These tasks or things can be:
-- control tasks
-- a task following
-- trajectories
-- motion planning problems
-- system level processes (like subscriptions, perceptual processes, etc)
-- interfaces (gamepad, orsviewers, etc)

The template for now is

auto* handle = Roopi::createX(...)
handle->setParameters
handle->run
handle->quit or Roopi::destroyX(handle)

Maybe plan:
-- first just control tasks & motion planning
-- make path following a thread
-- make logging just such a thread and 'handle'
-- next the system processes (perception threads)
-- the activities we already have

*/
//==============================================================================

struct Roopi_private {

  Access_typed<sensor_msgs::JointState> jointState;
  Access_typed<CtrlMsg> ctrl_ref;
  Access_typed<CtrlMsg> ctrl_obs;
  Access_typed<arr>     pr2_odom;

  //-- perception
  ACCESSname(FilterObjects, object_database)

  //-- controller process
  TaskControllerModule tcm;

  LoggingModule loggingModule;

  //-- ROS initialization
  RosInit rosInit;

  //TODO
  //-- RAP framework
  #if 0
  RelationalMachineModule rm;
  ActivitySpinnerModule aspin;
  ServiceRAP rapservice;
  #endif

  //-- sensor processes
  #if 0
  SubscribeTabletop tabletop_subscriber;
  SubscribeAlvar alvar_subscriber;
  Collector data_collector;
  Filter myFilter;
  PublishDatabase myPublisher;
  #endif

  //-- display and interaction
  GamepadInterface gamepad;
  //OrsViewer view;
  OrsPoseViewer ctrlView;

  //-- sync'ing with ROS
  Subscriber<sensor_msgs::JointState> subJointState;

  #if baxter
  SendPositionCommandsToBaxter spctb;
  #endif

  //PR2
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> subCtrl;
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pubCtrl;
  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       subOdom;

  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  Roopi_private()
    : jointState(NULL, "jointState"),
      ctrl_ref(NULL, "ctrl_ref"),
      ctrl_obs(NULL, "ctrl_obs"),
      pr2_odom(NULL, "pr2_odom"),
      tcm("pr2"),
      rosInit("Roopi"),
      ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld),
      subJointState("/robot/joint_states", jointState),
      #if baxter
      spctb(tcm.realWorld),
      #endif
      subCtrl("/marc_rt_controller/jointState", ctrl_obs),
      pubCtrl("/marc_rt_controller/jointReference", ctrl_ref),
      subOdom("/robot_pose_ekf/odom_combined", pr2_odom)
  {
    #if baxter
    if(mlr::getParameter<bool>("useRos", false)){
      nh = new ros::NodeHandle;
      pub = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
    }
    #endif

    threadOpenModules(true);
    mlr::wait(1.0);
    cout << "Go!" << endl;
  }

  ~Roopi_private(){
    threadCloseModules();
    cout << "bye bye" << endl;
  }
};


//==============================================================================

Roopi::Roopi()
  : s(new Roopi_private){
    planWorld = s->tcm.realWorld; // TODO something is wrong with planWorld
    mlr::timerStart(true); //TODO is that necessary? Is the timer global?
}

Roopi::~Roopi(){
  delete s;
}

//==============================================================================
//
// basic CtrlTask management

CtrlTask* Roopi::createCtrlTask(const char* name, TaskMap* map, bool active) {
  CtrlTask* ct = new CtrlTask(name, map);
  map->phi(ct->y, NoArr, tcm()->modelWorld.get()()); // initialize with the current value. TODO taskControllerModule updates these only if they are active
  ct->y_ref = ct->y;
  ct->setGains(0.0,0.0);
  ct->active = active;
  tcm()->ctrlTasks.set()->append(ct);
  return ct;
}

void Roopi::activateCtrlTask(CtrlTask* t, bool active){
  tcm()->ctrlTasks.set(), t->active=true;  //(mt) very unusual syntax.... check if that works!
}

void Roopi::destroyCtrlTask(CtrlTask* t) {
  tcm()->ctrlTasks.set()->removeValue(t);
  delete &t->map;
  delete t;
}


void Roopi::modifyCtrlTaskReference(CtrlTask* ct, const arr& yRef, const arr& yDotRef) {
  tcm()->ctrlTasks.writeAccess();
  ct->setTarget(yRef, yDotRef);
  tcm()->ctrlTasks.deAccess();
}

void Roopi::modifyCtrlTaskGains(CtrlTask* ct, const arr& Kp, const arr& Kd, const double maxVel, const double maxAcc) {
  tcm()->ctrlTasks.writeAccess();
  ct->setGains(Kp, Kd);
  ct->maxVel = maxVel;
  ct->maxAcc = maxAcc;
  tcm()->ctrlTasks.deAccess();
}

void Roopi::modifyCtrlC(CtrlTask* ct, const arr& C) {
  tcm()->ctrlTasks.writeAccess();
  ct->setC(C);
  tcm()->ctrlTasks.deAccess();
}

//CtrlTask* Roopi::_addQItselfCtrlTask(const arr& qRef, const arr& Kp, const arr& Kd) {
//  CtrlTask* ct = createCtrlTask("qItself", new TaskMap_qItself);
//  ct->setTarget(qRef);
//  ct->setC(ARR(1000.0));
//  ct->setGains(Kp, Kd);
//  //(mt): automatically activate the task??
//  //addCtrlTask(ct);
//  return ct;
//}

//CtrlTask* Roopi::_addDefaultCtrlTask(const char* name, const TaskMap_DefaultType type, const char* iShapeName, const ors::Vector& iVec, const char* jShapeName, const ors::Vector& jVec) {
//  CtrlTask* ct = createCtrlTask(name, new TaskMap_Default(type, tcm()->modelWorld.get()(), iShapeName, iVec, jShapeName, jVec), false);
//  return ct;
//}

TaskControllerModule* Roopi::tcm() {
  return &s->tcm;
}

//==============================================================================

arr Roopi::getJointState() {
  return tcm()->modelWorld.get()->getJointState();
}

arr Roopi::getTaskValue(CtrlTask* task) {
  arr y;
  task->map.phi(y, NoArr, tcm()->modelWorld.get()());
  return y;
}

//==============================================================================

void Roopi::syncPlanWorld() {
  arr qPlan;
  transferQbetweenTwoWorlds(qPlan, tcm()->modelWorld.get()->getJointState(), planWorld, tcm()->modelWorld.get()());
  planWorld.setJointState(qPlan);
}

ors::KinematicWorld& Roopi::getPlanWorld() {
  return planWorld;
}

void Roopi::followTaskTrajectory(CtrlTask* task, double executionTime, const arr& trajectory) {
  followTaskTrajectories(CtrlTaskL({task}), executionTime, {trajectory});
}

void Roopi::followTaskTrajectories(const CtrlTaskL& tasks, double executionTime, const arrA& trajY, const arrA& trajYDot, const arrA& trajYDDot) {
  //TODO not reasonable yet
  mlr::Array<mlr::Spline> ySplines;
  mlr::Array<mlr::Spline> yDotSplines;
  mlr::Array<mlr::Spline> yDDotSplines;

  for(uint i = 0; i < tasks.N; i++) { //Not sure if the iterator returns elements in FIFO order, therefore "for" loop
    if(trajY.N) {
      ySplines.append(mlr::Spline(trajY(i).d0, trajY(i)));
    }
    if(trajYDot.N) {
      yDotSplines.append(mlr::Spline(trajYDot(i).d0, trajYDot(i)));
    }
    if(trajYDDot.N) {
      yDDotSplines.append(mlr::Spline(trajYDDot(i).d0, trajYDDot(i)));
    }
  }

#if 1 //marc's version
  tcm()->ctrlTasks.writeAccess();
  for(CtrlTask *t:tcm()->ctrlTasks()) t->active=false;
  for(CtrlTask *t:tasks) t->active=true; //they were 'added' to the ctrlTasks list on creation!!
//  tcm()->verbose=1;
  tcm()->ctrlTasks.deAccess();
#else //danny's
  tcm()->ctrlTasks.set()->clear(); // TODO memory leak! The question is if it is a good idea to delete all tasks here, because one could still use them in the main.cpp
  addCtrlTasks(tasks);
#endif

  double startTime = mlr::timerRead();
  double time = 0.0;
  uint n = 0;
  cout << "start execution of trajectory" << endl;
  while(true) { //(mt) oh no! This goes full throttle!!
    time = mlr::timerRead() - startTime;
//    cout << time << endl;
    double s = time/executionTime;
    if(s > 1.) {
      cout << "finished execution of trajectory" << endl;
      break;
    }
    for(uint i = 0; i < tasks.N; i++) {
      modifyCtrlTaskReference(tasks(i), ySplines(i).eval(s));
    }
    n++;
  }
}

void Roopi::followQTrajectory(const Roopi_Path* path) {
  CtrlTask* ct = createCtrlTask("followQTrajectory", new TaskMap_qItself);
  ct->setC(ARR(1000.0));
  ct->setGains(ARR(30.0), ARR(5.0));
  followTaskTrajectory(ct, path->executionTime, path->path);
  destroyCtrlTask(ct);
}

Roopi_Path* Roopi::createPathInJointSpace(CtrlTask* task, double executionTime, bool verbose) {
  return createPathInJointSpace(CtrlTaskL({task}), executionTime, verbose);
}

Roopi_Path* Roopi::createPathInJointSpace(const CtrlTaskL& tasks, double executionTime, bool verbose) {
  Roopi_Path *path = new Roopi_Path(*this, executionTime);

  syncPlanWorld();
  MotionProblem MP(planWorld);

  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(MP.world), sumOfSqrTT);
  t->map.order=2; //acceleration task
  t->setCostSpecs(0, MP.T, {0.}, 1.0);

  t = MP.addTask("collisions", new CollisionConstraint(0.1), ineqTT);
  t->setCostSpecs(0., MP.T, {0.}, 1.0);
  t = MP.addTask("qLimits", new LimitsConstraint(0.1), ineqTT);
  t->setCostSpecs(0., MP.T, {0.}, 1.0);

  for(CtrlTask* ct : tasks) {
    t = MP.addTask(ct->name, &ct->map, sumOfSqrTT);
    t->setCostSpecs(MP.T-2, MP.T, ct->y_ref, 10.0); //TODO MP.T-how many? TODO ct->get_y_ref refactor!
  }

  path->path = MP.getInitialization();

  optConstrained(path->path , NoArr, Convert(MP), OPT(verbose=verbose, stopIters=100, damping=1., maxStep=1.,aulaMuInc=2, nonStrictSteps=5)); //TODO options
  if(verbose) MP.costReport();
  Graph result = MP.getReport();
  path->path.reshape(MP.T, path->path.N/MP.T);
  path->cost = result.get<double>({"total","sqrCosts"});
  path->constraints = result.get<double>({"total","constraints"});

  if(path->constraints < .1 && path->cost < 5.) {
    path->isGood=true;
  } else {
    path->isGood=false;
    cout << "No reasonable trajectory found!" << endl;
  }

  return path;
}

void Roopi::goToPosition(const arr& pos, const char* shape, double executionTime, bool verbose) {
  CtrlTask* ct = new CtrlTask("pos", new TaskMap_Default(posTMT, tcm()->modelWorld.get()(), shape));
  ct->setTarget(pos);
  auto* path = createPathInJointSpace(ct, executionTime, verbose);
  delete ct;
  followQTrajectory(path);
}


/*
void Roopi::waitConv(const CtrlTaskL& tasks){
  for(;;){
    mlr::wait(.03);
    bool allConv=true;
    for(CtrlTask *t:tasks) if(!t->isConverged()){ allConv=false; break; }
    if(allConv) return;
  }
}

CtrlTask* Roopi::modify(CtrlTask* t, const Graph& specs){
  s->tcm.ctrlTasks.writeAccess();
  t->set(specs);
#ifdef REPORT
  t->reportState(cout);
#endif
  s->tcm.ctrlTasks.deAccess();
  return t;
}
*/

#if 0 //(mt) no such low-level operations -- only createCtrlTask should be used!!
void Roopi::addCtrlTask(CtrlTask* ct) {
  tcm()->ctrlTasks.set()->append(ct);
}

void Roopi::addCtrlTasks(CtrlTaskL cts) {
  /*for(CtrlTask* t : cts) {
    addCtrlTask(t);
  }*/
  tcm()->ctrlTasks.set()->append(cts);
}
#endif
