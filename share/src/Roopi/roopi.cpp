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

  //-- logging
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

  ors::KinematicWorld* cu;

  //OrsViewer view;
  OrsPoseViewer* ctrlView;

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



  Roopi_private(ors::KinematicWorld& world)
    : jointState(NULL, "jointState"),
      ctrl_ref(NULL, "ctrl_ref"),
      ctrl_obs(NULL, "ctrl_obs"),
      pr2_odom(NULL, "pr2_odom"),
      tcm("pr2", world),
      rosInit("Roopi"),
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

    if(&world) {
      cu = new ors::KinematicWorld(world);
    } else {
      cu = new ors::KinematicWorld(tcm.realWorld);
    }

    if(mlr::getParameter<bool>("oldfashinedTaskControl")) {
      ctrlView = new OrsPoseViewer({"ctrl_q_real", "ctrl_q_ref"}, *cu);
    } else {
      ctrlView = new OrsPoseViewer({"ctrl_q_real"}, *cu);
    }

    threadOpenModules(true);
    mlr::wait(1.0);
    cout << "Go!" << endl;
  }

  ~Roopi_private(){
    threadCloseModules();
    delete ctrlView;
    delete cu;
    cout << "bye bye" << endl;
  }
};

//==============================================================================

Roopi::Roopi(ors::KinematicWorld& world)
  : s(new Roopi_private(world)){
  planWorld = s->tcm.realWorld; // TODO something is wrong with planWorld
  mlr::timerStart(true); //TODO is that necessary? Is the timer global?

  holdPositionTask = createCtrlTask("HoldPosition", new TaskMap_qItself);
  modifyCtrlTaskGains(holdPositionTask, 30.0, 5.0);
  modifyCtrlC(holdPositionTask, ARR(1000.0));
  activateCtrlTask(holdPositionTask);
}

Roopi::~Roopi(){
  delete s;
  delete holdPositionTask;
}

//==============================================================================
//
// basic CtrlTask management

CtrlTask* Roopi::createCtrlTask(const char* name, TaskMap* map, bool active) {
  CtrlTask* ct = new CtrlTask(name, map);
  map->phi(ct->y, NoArr, tcm()->modelWorld.get()()); // initialize with the current value. TODO taskControllerModule updates these only if they are active
  ct->y_ref = ct->y;
  ct->setGains(0.0,0.0);
  ct->setC(eye(ct->y_ref.d0)*1000.0); //TODO
  ct->active = active;
  tcm()->ctrlTasks.set()->append(ct);
  return ct;
}

void Roopi::activateCtrlTask(CtrlTask* t, bool reinitializeReferences){
  tcm()->ctrlTasks.writeAccess();
  if(reinitializeReferences) {
    cout << "basd" << endl;
    arr currentValue = getTaskValue(t);
    t->y = currentValue;
    t->y_ref = currentValue;
  }
  t->active = true;
  tcm()->ctrlTasks.deAccess();
}

void Roopi::deactivateCtrlTask(CtrlTask* t) {
  tcm()->ctrlTasks.writeAccess();
  t->active = false;
  tcm()->ctrlTasks.deAccess();
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

void Roopi::modifyCtrlTaskGains(CtrlTask* ct, const double& Kp, const double& Kd, const double maxVel, const double maxAcc) {
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

void Roopi::holdPosition() {
  tcm()->ctrlTasks.writeAccess();
  for(CtrlTask *t:tcm()->ctrlTasks()) t->active=false;
  tcm()->ctrlTasks.deAccess();

  /*CtrlTask* ct = createCtrlTask("HoldPosition", new TaskMap_qItself);
  modifyCtrlTaskGains(ct, 30.0, 5.0);
  modifyCtrlC(ct, ARR(1000.0));
  activateCtrlTask(ct);*/

  //modifyCtrlTaskReference(holdPositionTask, tcm()->modelWorld.get()->getJointState());
  activateCtrlTask(holdPositionTask, true);
}

void Roopi::releasePosition() {
  deactivateCtrlTask(holdPositionTask);
}

bool Roopi::converged(CtrlTask* ct, double tolerance) {
  bool conv = false;
  tcm()->ctrlTasks.readAccess();
  if(ct->isConverged(tolerance)) conv = true;
  tcm()->ctrlTasks.deAccess();
  return conv;
}

bool Roopi::waitForConv(CtrlTask* ct, double maxTime, double tolerance) {
  return waitForConv(CtrlTaskL({ct}), maxTime, tolerance);
}

bool Roopi::waitForConv(const CtrlTaskL& cts, double maxTime, double tolerance) {
  double startTime = mlr::timerRead();
  while(true) {
    bool allConv = true;
    tcm()->ctrlTasks.readAccess();
    for(CtrlTask* t : cts) {
      if(!t->isConverged(tolerance)) {
        allConv = false;
        break;
      } else {
        cout << t->name << " has converged" << endl;
      }
    }
    tcm()->ctrlTasks.deAccess();
    if(allConv) return true;
    double actTime = mlr::timerRead() - startTime;
    if(maxTime != -1 && actTime > maxTime) {
      cout << "not converged, timeout reached" << endl;
      return false;
    }
    mlr::wait(0.1);
  }
  return false;
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
  if(tcm()->oldfashioned && !tcm()->useRos) {
    return tcm()->modelWorld.get()->getJointState();
  }
  return tcm()->ctrl_obs.get()->q;
}

arr Roopi::getJointSign() {
  return tcm()->qSign.get();
}

arr Roopi::getTorques() {
  return tcm()->ctrl_obs.get()->u_bias;
}

arr Roopi::getFTLeft() {
  return tcm()->ctrl_obs.get()->fL;
}

arr Roopi::getFTRight() {
  return tcm()->ctrl_obs.get()->fR;
}

arr Roopi::getTaskValue(CtrlTask* task) {
  arr y;
  task->map.phi(y, NoArr, tcm()->modelWorld.get()()); //TODO or better with realWorld?
  return y;
}

//==============================================================================

void Roopi::syncPlanWorld() {
  arr qPlan;
  //this syncs with the real world, except for the case where no real world is available. TODO does this make sense?
  if(tcm()->oldfashioned && !tcm()->useRos) {
    transferQbetweenTwoWorlds(qPlan, tcm()->modelWorld.get()->getJointState(), planWorld, tcm()->modelWorld.get()());
  } else {
    transferQbetweenTwoWorlds(qPlan, tcm()->ctrl_obs.get()->q, planWorld, tcm()->modelWorld.get()());
  }
  planWorld.setJointState(qPlan);
}

ors::KinematicWorld& Roopi::getPlanWorld() {
  return planWorld;
}

double Roopi::getLimitConstraint(double margin) {
  LimitsConstraint limit(margin);
  arr y;
  syncPlanWorld();
  limit.phi(y, NoArr, planWorld);
  return y.first();
}

double Roopi::getCollisionConstraint(double margin) {
  CollisionConstraint collisions(margin);
  arr y;
  syncPlanWorld();
  collisions.phi(y, NoArr, planWorld);
  return y.first();
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
  destroyCtrlTask(ct); //TODO this is a bit unsmooth, because then no task is active anymore! Calling holdPosition directly afterwards works, bit is a bit unsmmooth
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

  t = MP.addTask("collisions", new CollisionConstraint(0.11), ineqTT);
  t->setCostSpecs(0., MP.T, {0.}, 1.0);
  t = MP.addTask("qLimits", new LimitsConstraint(0.1), ineqTT); //TODO!!!!!!!!!!!!!!! margin
  t->setCostSpecs(5, MP.T, {0.}, 1.0);

  for(CtrlTask* ct : tasks) {
    t = MP.addTask(ct->name, &ct->map, sumOfSqrTT);
    t->setCostSpecs(MP.T-5, MP.T, ct->y_ref, 5.0); //TODO MP.T-how many? TODO ct->get_y_ref refactor!
  }

  path->path = MP.getInitialization();

  optConstrained(path->path , NoArr, Convert(MP), OPT(verbose=verbose)); //TODO options
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

bool Roopi::goToPosition(const arr& pos, const char* shape, double executionTime, bool verbose) {
  CtrlTask* ct = new CtrlTask("pos", new TaskMap_Default(posTMT, tcm()->modelWorld.get()(), shape));
  ct->setTarget(pos);
  auto* path = createPathInJointSpace(ct, executionTime, verbose);
  delete ct;
  bool goodPath = false;
  if(path->isGood) {
    followQTrajectory(path);
    goodPath = true;
  }
  delete path;
  return goodPath;
}

bool Roopi::gotToJointConfiguration(const arr &jointConfig, double executionTime, bool verbose) {
  CtrlTask* ct = new CtrlTask("q", new TaskMap_qItself);
  ct->setTarget(jointConfig);
  Roopi_Path* path = createPathInJointSpace(ct, executionTime, verbose); //TODO why was there a auto*
  delete ct;
  bool goodPath = false;
  if(path->isGood) {
    followQTrajectory(path);
    goodPath = true;
  }
  delete path;
  return goodPath;
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
