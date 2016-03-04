#ifndef PR2DYNAMICSIMULATION_H
#define PR2DYNAMICSIMULATION_H

#include <Core/array.h>
#include <Core/module.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>

struct RTControllerSimulation : Module {
  Access_typed<CtrlMsg> ctrl_ref;
  Access_typed<CtrlMsg> ctrl_obs;
  Access_typed<ors::KinematicWorld> modelWorld;

  ors::KinematicWorld* world;
  double tau;
  bool gravity;

  RTControllerSimulation(double tau=0.01, bool gravity=false)
    : Module("DynmSim", tau)
    , ctrl_ref(this, "ctrl_ref")
    , ctrl_obs(this, "ctrl_obs")
    , modelWorld(this, "modelWorld")
    , tau(tau)
    , gravity(gravity) {}
  virtual ~RTControllerSimulation() {}

  void open();
  void step();
  void close(){}
};

#endif // PR2DYNAMICSIMULATION_H
