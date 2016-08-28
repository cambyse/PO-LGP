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

struct RoopiSystem {

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

  RoopiSystem()
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

  ~RoopiSystem(){
    threadCloseModules();
    cout << "bye bye" << endl;
  }
};


Roopi::Roopi()
  : s(new RoopiSystem){
    planWorld = s->tcm.realWorld; // TODO something is wrong with planWorld
    mlr::timerStart(true); //TODO is that necessary? Is the timer global?
}

Roopi::~Roopi(){
  delete s;
}

TaskControllerModule* Roopi::tcm() {
  return &s->tcm;
}

void Roopi::addCtrlTask(CtrlTask* ct) {
  tcm()->ctrlTasks.set()->append(ct);
}

void Roopi::addCtrlTasks(CtrlTaskL cts) {
  /*for(CtrlTask* t : cts) {
    addCtrlTask(t);
  }*/
  tcm()->ctrlTasks.set()->append(cts);
}

CtrlTask* Roopi::addQItselfCtrlTask(const arr& qRef, const arr& Kp, const arr& Kd) {
  CtrlTask* ct = new CtrlTask("qItself", new TaskMap_qItself);
  ct->setTarget(qRef);
  ct->setC(ARR(1000.0));
  ct->setGains(Kp, Kd);
  addCtrlTask(ct);
  return ct;
}

CtrlTask* Roopi::addDefaultCtrlTask(const char* name, const TaskMap_DefaultType type, const char* iShapeName, const ors::Vector& iVec, const char* jShapeName, const ors::Vector& jVec) {
  CtrlTask* ct = createCtrlTask(name, new TaskMap_Default(type, tcm()->modelWorld.get()(), iShapeName, iVec, jShapeName, jVec), false);
  return ct;
}

CtrlTask* Roopi::createCtrlTask(const char* name, TaskMap* map, bool active) {
  CtrlTask* ct = new CtrlTask(name, map);
  ct->active = active;
  map->phi(ct->y, NoArr, tcm()->modelWorld.get()()); // initialize with the current value. TODO taskControllerModule updates these only if they are active
  ct->setGains(0.0,0.0);
  addCtrlTask(ct);
  return ct;
}

void Roopi::modifyCtrlActive(CtrlTask* ct, bool active) {
  tcm()->ctrlTasks.writeAccess();
  ct->active = active;
  tcm()->ctrlTasks.deAccess();
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

void Roopi::destroyCtrlTask(CtrlTask* t) {
  tcm()->ctrlTasks.set()->removeValue(t);
  delete &t->map;
  delete t;
}

arr Roopi::getJointState() {
  return tcm()->modelWorld.get()->getJointState();
}

arr Roopi::getTaskValue(CtrlTask* task) {
  arr y;
  task->map.phi(y, NoArr, tcm()->modelWorld.get()());
  return y;
}

void Roopi::syncPlanWorld() {
  arr qPlan;
  transferQbetweenTwoWorlds(qPlan, tcm()->modelWorld.get()->getJointState(), planWorld, tcm()->modelWorld.get()());
  planWorld.setJointState(qPlan);
}

ors::KinematicWorld& Roopi::getPlanWorld() {
  return planWorld;
}

void Roopi::followTaskTrajectory(CtrlTask* task, double executionTime, const arr& trajectory) {
  followTaskTrajectories({task}, executionTime, {trajectory});
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

  tcm()->ctrlTasks.set()->clear(); // TODO memory leak! The question is if it is a good idea to delete all tasks here, because one could still use them in the main.cpp
  addCtrlTasks(tasks);

  double startTime = mlr::timerRead();
  double time = 0.0;
  uint n = 0;
  cout << "start execution of trajectory" << endl;
  while(true) {
    double s;
    if(time < executionTime) {
      s = time/executionTime;
    } else {
      s = 1;
      cout << "finished execution of trajectory" << endl;
      break;
    }
    for(uint i = 0; i < tasks.N; i++) {
      modifyCtrlTaskReference(tasks(i), ySplines(i).eval(s));
    }
    n++;
    time = mlr::timerRead() - startTime;
    //cout << time << endl;
  }
}

void Roopi::followQTrajectory(double executionTime, const arr& trajectory) {
  CtrlTask* ct = addQItselfCtrlTask(getJointState()); //TODO murks, because here a task is added and then in followTaskTrajectory removed and then the same again added
  followTaskTrajectory(ct, executionTime, trajectory);
}

void Roopi::goToTaskMotionPlannerJointSpace(CtrlTask* task, double executionTime, bool verbose) {
  goToTasksMotionPlannerJointSpace({task}, executionTime, verbose);
}

void Roopi::goToTasksMotionPlannerJointSpace(const CtrlTaskL& tasks, double executionTime, bool verbose) {
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

  arr traj = MP.getInitialization();
  traj.reshape(MP.T,MP.world.getJointStateDimension());

  optConstrained(traj, NoArr, Convert(MP), OPT(verbose=verbose, stopIters=100, damping=1., maxStep=1.,aulaMuInc=2, nonStrictSteps=5)); //TODO options
  if(verbose) MP.costReport();
  Graph result = MP.getReport();
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if(constraints < .1 && cost < 5.) {
    followQTrajectory(executionTime, traj);
  } else {
    cout << "No reasonable trajectory found!" << endl;
  }
}

void Roopi::goToPosition(const arr& pos, const char* shape, double executionTime, bool verbose) {
  CtrlTask* ct = new CtrlTask("pos", new TaskMap_Default(posTMT, tcm()->modelWorld.get()(), shape));
  ct->setTarget(pos);
  goToTaskMotionPlannerJointSpace(ct, executionTime, verbose);
  delete ct;
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
