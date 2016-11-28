#pragma once

#include <Core/array.h>
#include <Core/graph.h>
#include <Motion/taskMaps.h>

struct CtrlTask;
typedef mlr::Array<CtrlTask*> CtrlTaskL;
struct RelationalMachineModule;

struct MyBaxter{
  struct MyBaxter_private* s;

  CtrlTaskL activeTasks;
  mlr::KinematicWorld testWorld;

  MyBaxter(const bool report = false);
  ~MyBaxter();

  //-- add & modify tasks
  CtrlTask* task(const Graph& specs);
  CtrlTask* task(const char* name,
                 const Graph& specs);

  CtrlTask* task(const char* name,
                 TaskMap* map,
                 double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask* task(const char* name,
                 TaskMap* map);
  CtrlTask* task(const char* name,
                 TaskMap_DefaultType type,
                 const char* iShapeName, const mlr::Vector& ivec,
                 const char* jShapeName, const mlr::Vector& jvec,
                 const arr& target,
                 double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask* modify(CtrlTask* t, const Graph& specs);
  CtrlTask* modifyTarget(CtrlTask* t, const arr& target);

  //-- wait for & stop tasks
  void stop(const CtrlTaskL& tasks);
  void stopAll();
  void waitConv(const CtrlTaskL& tasks);
  bool testConv(const CtrlTaskL& tasks, const double waitSecs=10);
  bool testRealConv(const CtrlTaskL& tasks, const double waitSecs=10);

  //-- get object information
  uint reportPerceptionObjects();
  void reportJointState();
  arr getEfforts();
  arr getJointState();
  void getState(arr& q, arr& qdot, arr& u);
  double setTestJointState(const arr& q);
  double updateLockbox(const mlr::Transformation& tf);
  void getEquationOfMotion(arr& M, arr& F);

  void setRealWorld(arr& q);

  //-- get position closest cluster
  mlr::Vector closestCluster();
  mlr::Vector arPose();

  void disablePosControl();
  void enablePosControl();

  void enableTotalTorqueMode();
  void disableTotalTorqueMode();
  void publishTorque(const arr& u, const char* prefix="right_");

  const mlr::KinematicWorld& getKinematicWorld();
  const mlr::KinematicWorld& getModelWorld();

  arr getJointLimits();
  double getCollisionScalar();

  void setLimits();
  void releaseLimits();

  void grip();
  void grip(const bool grip, const bool sim = false);

  bool isGripping = false;

  CtrlTask* gripTask;
  //-- inner access
  struct TaskControllerModule& getTaskControllerModule();

  //-- info
  arr q0();

  bool reportState;
};
