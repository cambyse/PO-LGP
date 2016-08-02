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
  ors::KinematicWorld testWorld;

  MyBaxter();
  ~MyBaxter();

  //-- add & modify tasks
  CtrlTask* task(const Graph& specs);
  CtrlTask* task(const char* name,
                 TaskMap* map,
                 double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask* task(const char* name,
                 TaskMap* map);
  CtrlTask* task(const char* name,
                 TaskMap_DefaultType type,
                 const char* iShapeName, const ors::Vector& ivec,
                 const char* jShapeName, const ors::Vector& jvec,
                 const arr& target,
                 double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask* modify(CtrlTask* t, const Graph& specs);
  CtrlTask* modifyTarget(CtrlTask* t, const arr& target);

  //-- wait for & stop tasks
  void stop(const CtrlTaskL& tasks);
  void waitConv(const CtrlTaskL& tasks);

  //-- get object information
  uint reportPerceptionObjects();
  void reportJointState();
  arr getEfforts();
  arr getJointState();
  void getState(arr& q, arr& qdot, arr& u);
  double setTestJointState(const arr& q);
  double updateLockbox(const ors::Transformation& tf);
  void getEquationOfMotion(arr& M, arr& F);

  void setRealWorld(arr& q);

  //-- get position closest cluster
  ors::Vector closestCluster();
  ors::Vector arPose();

  void disablePosControl();
  void enablePosControl();

  void enableTotalTorqueMode();
  void disableTotalTorqueMode();
  void publishTorque(const arr& u, const char* prefix="right_");

  const ors::KinematicWorld& getKinematicWorld();
  const ors::KinematicWorld& getModelWorld();

  arr getJointLimits();
  double getCollisionScalar();

  void grip();
  void grip(const bool grip, const bool sim = false);

  bool isGripping = false;

  //-- inner access
  struct TaskControllerModule& getTaskControllerModule();

  //-- info
  arr q0();
};
