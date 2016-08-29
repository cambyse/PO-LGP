#pragma once

#include <Core/array.h>
#include <Core/graph.h>
#include <Motion/taskMaps.h>
#include <Control/taskController.h>


struct CtrlTask;
struct Roopi_Path;
typedef mlr::Array<CtrlTask*> CtrlTaskL;
//struct RelationalMachineModule;

//==============================================================================

struct Roopi {
  struct Roopi_private* s;

  CtrlTaskL activeTasks;
  ors::KinematicWorld planWorld; ///< kinematic world for pose optimization, external degrees of freedom etc.

  Roopi();
  ~Roopi();

  //-- control tasks

  /// creates a new CtrlTask; pass a 'newed' map as argument, it will be deleted later; after creation it is inactive
  CtrlTask *createCtrlTask(const char* name, TaskMap* map, bool active=false);

  /// activate a control task after you've set parameters, etc
  void activateCtrlTask(CtrlTask* t, bool active=true);

  /// removes a CtrlTask and deletes also the task map
  void destroyCtrlTask(CtrlTask* t);

  void modifyCtrlTaskReference(CtrlTask* ct, const arr& yRef, const arr& yDotRef = NoArr);
  void modifyCtrlTaskGains(CtrlTask* ct, const arr& Kp, const arr& Kd, const double maxVel = 0.0, const double maxAcc = 0.0);
  void modifyCtrlC(CtrlTask* ct, const arr& C);



  // low-level ctr - use is discouraged!!
  struct TaskControllerModule* tcm(); //low-level access of the tcm - really necessary?
//  void addCtrlTask(CtrlTask* ct); ///< adds CtrlTask directly to the taskController
//  void addCtrlTasks(CtrlTaskL cts); ///< adds multiple CtrlTasks directly to the taskController

  // shortcuts to crate standard CtrlTasks (mt) I'd remove this for now
  //  /// adds qItself task to the taskController
  //  CtrlTask* _addQItselfCtrlTask(const arr& qRef, const arr& Kp = ARR(30.0), const arr& Kd = ARR(5.0));

  //  /// creates inactive default task with zero gains and adds it to the taskController
  //  CtrlTask* _addDefaultCtrlTask(const char* name,
  //                               const TaskMap_DefaultType type,
  //                               const char* iShapeName, const ors::Vector& iVec = NoVector,
  //                               const char* jShapeName = NULL, const ors::Vector& jVec = NoVector);

  //-- trajectory tasks

  void followTaskTrajectory(CtrlTask* task, double executionTime, const arr& trajectory);
  void followTaskTrajectories(const CtrlTaskL& tasks, double executionTime, const arrA& trajY, const arrA& trajYDot = {}, const arrA& trajYDDot = {});
  void followQTrajectory(const Roopi_Path* path);

  //-- planning tasks

  Roopi_Path* createPathInJointSpace(CtrlTask* task, double executionTime, bool verbose = false);
  Roopi_Path* createPathInJointSpace(const CtrlTaskL& tasks, double executionTime, bool verbose = false);

  //-- macros

  /// move "shape" to "pos" using a motion planner.
  void goToPosition(const arr& pos, const char* shape, double executionTime, bool verbose = false);

  /// move to joint configuration using a motion planner
  void gotToJointConfiguration(const arr& jointConfig, double executionTime, bool verbose = false);

  //-- low-level access

  arr getJointState();
  /// get current value of the underlying task map
  arr getTaskValue(CtrlTask* task);

  /// sync the joint configuration of the model world into the planWorld
  void syncPlanWorld();
  ors::KinematicWorld& getPlanWorld();


  //-- TODO

  //getModelWorld() what return type?

//  //-- wait for & stop tasks
//  void stop(const CtrlTaskL& tasks);
//  void waitConv(const CtrlTaskL& tasks);

};

//==============================================================================

struct Roopi_CtrlTask{
  Roopi &roopi;
  CtrlTask *task;
  Roopi_CtrlTask(Roopi& r, CtrlTask *t) : roopi(r), task(t) {}
};

//==============================================================================

struct Roopi_Path{
  Roopi &roopi;
  arr path;
  double executionTime;
  double cost;
  double constraints;
  bool isGood;
  Roopi_Path(Roopi& r, double executionTime) : roopi(r), executionTime(executionTime), isGood(false){}
};
