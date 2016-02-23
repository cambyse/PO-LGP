#ifndef PR2DYNAMICSIMULATION_H
#define PR2DYNAMICSIMULATION_H

#include <Core/array.h>
#include <Core/module.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>

struct RTControllerSimulation : Module {
  ACCESSnew(CtrlMsg, ctrl_ref)
  ACCESSnew(CtrlMsg, ctrl_obs)
  ACCESSnew(ors::KinematicWorld, modelWorld)

  ors::KinematicWorld* world;
  double tau;
  bool gravity;

  RTControllerSimulation(double tau=0.01, bool gravity=false) : Module("DynmSim", tau), tau(tau), gravity(gravity) {}
  virtual ~RTControllerSimulation() {}

  virtual void open();
  virtual void step();
};

#endif // PR2DYNAMICSIMULATION_H
