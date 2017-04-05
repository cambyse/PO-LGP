#pragma once

#include <Core/array.h>
#include <Core/graph.h>
#include <Kin/taskMaps.h>

struct CtrlTask;
typedef mlr::Array<CtrlTask*> CtrlTaskL;
struct RelationalMachineModule;

struct Roopi{
  struct Roopi_private* s;

  CtrlTaskL activeTasks;
  mlr::KinematicWorld testWorld;

  Roopi();
  ~Roopi();

  struct TaskControlThread *tcm();
  //-- add & modify tasks


  CtrlTask *createCtrlTask(const char* name, TaskMap* map); //map should be 'newed' when calling this -> will be deleted on destroyCtrlTask
  //returns INACTIVE task!

  void destroyCtrlTask(CtrlTask* t); //deletes also the map

  //getModelWorld

  //followTaskTrajectory(const arrL& trajectory, CtrlTaskL& t);

//  CtrlTask* task(const Graph& specs);
//  CtrlTask* task(const char* name,
//                 TaskMap_DefaultType type,
//                 const char* iShapeName, const mlr::Vector& ivec,
//                 const char* jShapeName, const mlr::Vector& jvec,
//                 const arr& target,
//                 double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask* modify(CtrlTask* t, const Graph& specs);
  CtrlTask* modifyTarget(CtrlTask* t, const arr& target);

//  //-- wait for & stop tasks
  void stop(const CtrlTaskL& tasks);
  void waitConv(const CtrlTaskL& tasks);

//  //-- get object information
//  uint reportPerceptionObjects();
//  void reportJointState();
//  arr getEfforts();
//  arr getJointState();
//  double setTestJointState(const arr& q);
//  void getEquationOfMotion(arr& M, arr& F);


//  //-- get position closest cluster
//  mlr::Vector closestCluster();
//  mlr::Vector arPose();

//  void disablePosControl();
//  void enablePosControl();

//  void enableTotalTorqueMode();
//  void disableTotalTorqueMode();
//  void publishTorque(const arr& u, const char* prefix="right_");

//  const mlr::KinematicWorld& getKinematicWorld();

//  arr getJointLimits();
//  double getCollisionScalar();


 //-- planning and optimization in a model world


  //-- info
//  arr q0();
};
