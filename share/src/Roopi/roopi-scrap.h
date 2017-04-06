#pragma once

#include <Core/array.h>
#include <Core/graph.h>
#include <Core/module.h>
#include <Kin/taskMaps.h>
#include <Control/taskControl.h>


//==============================================================================
void example(){


  auto task = roopi.create(...);

  task.waitFor...();

  roopi.stop(task);
  task.stop();


  auto path = roopi.createPath(..);

  roopi.waitForStop(path);

  switch(path.status){//more semantic status, different for each particular activity

  }


  auto follower = roopi.followQTrajectory(path);

  roopi.waitForConv(follower);

}
//==============================================================================



struct CtrlTask;
struct Roopi_Path;
struct TaskReferenceInterpolAct;
typedef mlr::Array<CtrlTask*> CtrlTaskL;
//struct RelationalMachineModule;

//==============================================================================

struct Roopi_Activity{
  Signaler status;
};

//==============================================================================

struct Roopi {
  struct Roopi_private* s;

  CtrlTaskL activeTasks;
  mlr::KinematicWorld planWorld; ///< kinematic world for pose optimization, external degrees of freedom etc.

  CtrlTask* holdPositionTask;

  Roopi(mlr::KinematicWorld& world = NoWorld);
  ~Roopi();

  
  void loadKinematics(const char* filename, const char* variableKey="kinematics");

  void startTaskController();
  void startControllerLog();
  void startRosCom();
  void startObjectFilter(bool alvar=true, bool tabletop=true);
  void startGamepadTasker();

  //-- display and interaction
  GamepadInterface gamepad;

  mlr::KinematicWorld* cu;

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


  //-- general control flow activities


  /// wait for all given CtrlTasks to be converged
  bool waitForConv(Activity& ct, double maxTime = -1, double tolerance = 1e-2);
  bool waitForConv(const ActivityL& cts, double maxTime = -1, double tolerance = 1e-2);

  /// wait for a CtrlTask to be at a specific state;
  bool waitForConvTo(CtrlTask* ct, const arr& desState, double maxTime = -1, double tolerance = 1e-2);


  //-- control modes (examples)

  void en/disableBaseMotion();
  void en/disableOldFashioned();
  void en/disableSomeJoint(..);

  void disablePosControl();
  void enablePosControl();

  void enableTotalTorqueMode();
  void disableTotalTorqueMode();

  //-- control tasks

  /// creates a new CtrlTask; pass a 'newed' map as argument, it will be deleted later; after creation it is inactive
  Roopi_CtrlTask createCtrlTask(const char* name, TaskMap* map, bool active=false);



  //-- higher-level behaviors (they create control tasks themselves)

  /// move "shape" to "pos" using a motion planner.
  bool goToPosition(const arr& pos, const char* shape, double executionTime, bool verbose = false);

  /// move to joint configuration using a motion planner
  bool gotToJointConfiguration(const arr& jointConfig, double executionTime, bool verbose = false);

  /// holds all joints in position. Desactivates all other tasks
  void holdPosition();
  /// release hold position task. Has no effect on other tasks
  void releasePosition();


  //-- planning tasks

  //these should start an optimization thread
  Roopi_Path createPathInJointSpace(CtrlTask* task, double executionTime, bool verbose = false);
  Roopi_Path createPathInJointSpace(const CtrlTaskL& tasks, double executionTime, bool verbose = false);


  Roopi_PathFollower followQTrajectory(const Roopi_Path* path);
  void followTaskTrajectory(CtrlTask* task, double executionTime, const arr& trajectory);
  void followTaskTrajectories(const CtrlTaskL& tasks, double executionTime, const arrA& trajY, const arrA& trajYDot = {}, const arrA& trajYDDot = {});


  //-- trajectory tasks (up to Danny)

  void interpolateToReferenceThread(CtrlTask *task, double executionTime, const arr &reference, const arr &start);

  TaskReferenceInterpolAct* createTaskReferenceInterpolAct(const char* name, CtrlTask* task);
  void interpolateToReference(TaskReferenceInterpolAct* t, double executionTime, const arr& reference, const arr& start = NoArr);
  bool waitForFinishedTaskReferenceInterpolAct(TaskReferenceInterpolAct* t, double maxTime = -1);

  void interpolateToReference(CtrlTask* task, double executionTime, const arr& reference, const arr& start = NoArr);



  //-- data access

  arr getJointState();
  arr getJointSign();
  arr getTorques();
  arr getFTLeft();
  arr getFTRight();

  /// get current value of the underlying task map
  arr getTaskValue(CtrlTask* task);

  /// sync the joint configuration of the model world into the planWorld
  void syncPlanWorld();
  mlr::KinematicWorld& getPlanWorld();

  double getLimitConstraint(double margin = 0.05);
  double getCollisionConstraint(double margin = 0.1);

  // low-level ctr - use is discouraged!!
  struct TaskControlThread* tcm(); //low-level access of the tcm - really necessary? Danny: yes

  //-- TODO

  //getModelWorld() what return type?

//  //-- wait for & stop tasks
//  void stop(const CtrlTaskL& tasks);
//  void waitConv(const CtrlTaskL& tasks);

};

//==============================================================================

struct Roopi_CtrlTask : Roopi_Activity{
  Roopi &roopi;
  CtrlTask *task;
  Roopi_CtrlTask(Roopi& r, CtrlTask *t) : roopi(r), task(t) {
  }

  ~Roopi_CtrlTask(){
    //destroy control task
  }

  /// activate a control task after you've set parameters, etc
  void activate(bool reinitializeReferences = false);
  /// deactivate a control task
  void deactivate(CtrlTask* t);

  void setConvergenceCriteria(..);

  /// removes a CtrlTask and deletes also the task map
  void destroy(CtrlTask* t);

  /// modifies CtrlTasks
  void modifyReference(const arr& yRef, const arr& yDotRef = NoArr);
  void modifyGains(const arr& Kp, const arr& Kd, const double maxVel = 0.0, const double maxAcc = 0.0);
  void modifyGains(const double& Kp, const double& Kd, const double maxVel = 0.0, const double maxAcc = 0.0);
  void modifyCtrlC(const arr& C);
  /// force related
  void modifyForceRef(const arr& fRef);
  void modifyForceAlpha(double fAlpha);
  void modifyForceGamma(double fGamma);
  void modifyForce(const arr& fRef, const double& fAlpha, const double& fGamma);

  /// returns whether a CtrlTask has converged
  bool converged(CtrlTask* ct, double tolerance = 1e-2);


};

//==============================================================================

struct Roopi_Path : Roopi_Activity, Thread{
  Roopi &roopi;
  arr path;
  double executionTime;
  double cost;
  double constraints;
  bool isGood;
  Roopi_Path(Roopi& r, double executionTime) : roopi(r), executionTime(executionTime), isGood(false){}
};

//==============================================================================

struct Roopi_PathFollower{

};

//==============================================================================

struct TaskReferenceInterpolAct : Module {
  Roopi& roopi;
  CtrlTask* task;
  double executionTime;
  arr reference;
  arr initialRef;
  double startTime;

  TaskReferenceInterpolAct(Roopi& roopi, const char* name, CtrlTask* task);
  ~TaskReferenceInterpolAct();

  void startExecution(double executionTime, const arr& reference, const arr& startState = NoArr);
  void stopExecution();

  void open();
  void step();
  void close();
};

