#ifndef PR2DYNAMICSIMULATION_H
#define PR2DYNAMICSIMULATION_H

#include <Core/array.h>
#include <Core/module.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>

struct DynamicSimulation : Module {
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  DynamicSimulation() : Module("DynmSim") {}
  virtual ~DynamicSimulation() {}
  virtual void step();

  ors::KinematicWorld* world;

  bool gravity;

  arr ftErrInt;

  void initializeSimulation(ors::KinematicWorld* world, bool gravity = false);
  void startSimulation(bool start = true);

  void setWorld(ors::KinematicWorld* world);
  void setGravity(bool gravity);
  void setQ();
};

#endif // PR2DYNAMICSIMULATION_H
