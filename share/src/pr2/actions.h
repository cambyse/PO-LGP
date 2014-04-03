#include "actionMachine_internal.h"

/** @file This file contains implementations of GroundedActions.  */

//===========================================================================
struct CoreTasks : GroundedAction {
  CoreTasks();

  /// @name Inherited stuff
  static Symbol symbol;
  virtual Symbol& getSymbol() { return symbol; }
  virtual void initYourself(ActionMachine& actionMachine);
};

//===========================================================================
struct MoveEffTo : GroundedAction {
  MT::String effName;
  arr effPos;

  MoveEffTo(const char* effName, const arr& effPos);

  /// @name Inherited/overwritten stuff
  static Symbol symbol;
  virtual Symbol& getSymbol() { return symbol; }

  virtual void initYourself(ActionMachine& actionMachine);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct AlignEffTo : GroundedAction {
  MT::String effName;
  arr effPos;
  arr alginPos; // TODO what is this? Find a proper name.

  AlignEffTo(const char* effName, const arr& effPos, const arr& alignPos);

  /// @name Inherited stuff
  static Symbol symbol;
  virtual Symbol& getSymbol() {return symbol;}
  virtual void initYourself(ActionMachine& actionMachine);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct PushForce : GroundedAction {
  MT::String effName;
  ors::Vector forceVec;
  arr poseArg2; // TODO what is this? Find a proper name.

  PushForce(const char* effName, ors::Vector forceVec, arr poseArg2);

  /// @name Inherited stuff
  static Symbol symbol;
  virtual Symbol& getSymbol() {return symbol;}
  virtual void initYourself(ActionMachine& actionMachine);
};
