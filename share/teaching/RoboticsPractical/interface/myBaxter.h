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

  MyBaxter();
  ~MyBaxter();

  //-- add & modify tasks
  CtrlTask* task(const Graph& specs);
  CtrlTask* task(const char* name,
                 DefaultTaskMapType type,
                 const char* iShapeName, const ors::Vector& ivec,
                 const char* jShapeName, const ors::Vector& jvec,
                 const arr& target,
                 double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask* modify(CtrlTask* t, const Graph& specs);
  CtrlTask* modifyTarget(CtrlTask* t, const arr& target);

  //-- wait for & stop tasks
  void stop(const CtrlTaskL& tasks);
  void waitConv(const CtrlTaskL& tasks);

  //-- info
  arr q0();
};
