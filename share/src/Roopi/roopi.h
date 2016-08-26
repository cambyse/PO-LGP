#pragma once

#include <Core/array.h>
#include <Core/graph.h>
#include <Motion/taskMaps.h>


struct CtrlTask;
typedef mlr::Array<CtrlTask*> CtrlTaskL;
//struct RelationalMachineModule;

struct Roopi {
  struct RoopiSystem* s;

  CtrlTaskL activeTasks;
  ors::KinematicWorld planWorld; ///< kinematic world for pose optimization external degrees of freedom etc.

  Roopi();
  ~Roopi();

  struct TaskControllerModule* tcm();

  /// adds CtrlTask directly to the taskController
  void addCtrlTask(CtrlTask* ct);
  /// adds multiple CtrlTasks directly to the taskController
  void addCtrlTasks(CtrlTaskL cts);

  /// adds qItself task to the taskController
  CtrlTask* addQItselfCtrlTask(const arr& qRef, const arr& Kp = ARR(30.0), const arr& Kd = ARR(5.0));

  /// creates inactive default task with zero gains and adds it to the taskController
  CtrlTask* addDefaultCtrlTask(const char* name,
                               const TaskMap_DefaultType type,
                               const char* iShapeName, const ors::Vector& iVec = NoVector,
                               const char* jShapeName = NULL, const ors::Vector& jVec = NoVector);

  /// creates (default inactive) CtrlTask
  CtrlTask* createCtrlTask(const char* name, TaskMap* map, bool active = false); //create map with "new" directly here, because roopi deletes it when calling destroyCtrlTask

  void modifyCtrlActive(CtrlTask* ct, bool active);
  void modifyCtrlTaskReference(CtrlTask* ct, const arr& yRef, const arr& yDotRef = NoArr);
  void modifyCtrlTaskGains(CtrlTask* ct, const arr& Kp, const arr& Kd, const double maxVel = 0.0, const double maxAcc = 0.0);
  void modifyCtrlC(CtrlTask* ct, const arr& C);

  /// removes a CtrlTask and deletes also the task map
  void destroyCtrlTask(CtrlTask* t);

  arr getJointState();
  /// get current value of the underlying task map
  arr getTaskValue(CtrlTask* task);

  /// sync the joint configuration of the model world into the planWorld
  void syncPlanWorld();
  ors::KinematicWorld& getPlanWorld();

  //-- not properly realized yet
  void followTaskTrajectory(CtrlTask* task, double executionTime, const arr& trajectory);
  void followTaskTrajectories(const CtrlTaskL& tasks, double executionTime, const arrA& trajY, const arrA& trajYDot = {}, const arrA& trajYDDot = {});
  void followQTrajectory(double executionTime, const arr& trajectory);

  void goToTaskMotionPlannerJointSpace(CtrlTask* task, double executionTime, bool verbose = false);
  void goToTasksMotionPlannerJointSpace(const CtrlTaskL& tasks, double executionTime, bool verbose = false);

  /// move "shape" to "pos" using a motion planner.
  void goToPosition(const arr& pos, const char* shape, double executionTime, bool verbose = false);


  //-- TODO

  //getModelWorld() what return type?

//  //-- wait for & stop tasks
//  void stop(const CtrlTaskL& tasks);
//  void waitConv(const CtrlTaskL& tasks);

};
