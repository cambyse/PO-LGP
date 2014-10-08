#include "actionMachine_internal.h"

/** @file This file contains implementations of GroundedActions.  */

//===========================================================================
struct CoreTasks : GroundedAction {
  CoreTasks(ActionMachine& actionMachine);
};

//===========================================================================
struct MoveEffTo : GroundedAction {
  MT::String effName;
  arr effPos;

  MoveEffTo(ActionMachine& actionMachine, const char* effName, const arr& effPos);

  /// @name Inherited/overwritten stuff
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct PoseTo : GroundedAction {
  MT::String effName;
  arr effPos;
  arr orientation;

  PoseTo(ActionMachine& actionMachine, const char* effName, const arr& effPos, const arr& orientation);

  /// @name Inherited/overwritten stuff
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct AlignEffTo : GroundedAction {
  MT::String effName;
  arr effPos;
  arr alginPos; // TODO what is this? Find a proper name.

  AlignEffTo(ActionMachine& actionMachine, const char* effName, const arr& effPos, const arr& alignPos);

  /// @name Inherited stuff
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct OrientationQuat : GroundedAction {
  MT::String effName;
  arr orientation;

  OrientationQuat(ActionMachine& actionMachine, const char* effName, const arr& orientation);

  /// @name Inherited stuff
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct SetQ : GroundedAction {
  MT::String effName;
  int jointID;
  double jointPos;

  SetQ(ActionMachine& actionMachine, const char* effName, int jointID, double jointPos);

  /// @name Inherited stuff
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct PushForce : GroundedAction {
  MT::String effName;
  arr forceVec;
//  arr poseArg2; // TODO what is this? Find a proper name.

  PushForce(ActionMachine& actionMachine, const char* effName, arr forceVec/*, arr poseArg2*/);

  /// @name Inherited stuff
  bool finishedSuccess(ActionMachine& M);
};
