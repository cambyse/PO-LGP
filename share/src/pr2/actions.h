#include "actionMachine_internal.h"

/** @file This file contains implementations of GroundedActions.  */

//===========================================================================
struct CoreTasks : GroundedAction {
  CoreTasks();

  /// @name Inherited stuff
  virtual void initYourself(ActionMachine& actionMachine);
};

//===========================================================================
struct MoveEffTo : GroundedAction {
  MT::String effName;
  arr effPos;

  MoveEffTo(const char* effName, const arr& effPos);

  /// @name Inherited/overwritten stuff
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
  virtual void initYourself(ActionMachine& actionMachine);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct SetQ : GroundedAction {
  MT::String effName;
  int jointID;
  double jointPos;

  SetQ(const char* effName, int jointID, double jointPos)
    : GroundedAction("SetQ", 0)
    , effName(effName)
    , jointID(jointID)
    , jointPos(jointPos)
  {
    SymbolL::memMove=true;
    PDtaskL::memMove=true;
  };

  /// @name Inherited stuff
  virtual void initYourself(ActionMachine& actionMachine) {
    auto task = actionMachine.s->MP.addPDTask(
        effName, 2, .8, new DefaultTaskMap(qSingleTMT, jointID));
    task->setTarget({jointPos});
    tasks.append(task);
  }

  virtual bool finishedSuccess(ActionMachine& M) {
    PDtask *task=tasks(0);
    return (task->y.N == task->y_ref.N &&
            maxDiff(task->y, task->y_ref) < 1e-1);
  }
};

//===========================================================================
struct PushForce : GroundedAction {
  MT::String effName;
  arr forceVec;
//  arr poseArg2; // TODO what is this? Find a proper name.

  PushForce(const char* effName, arr forceVec/*, arr poseArg2*/);

  /// @name Inherited stuff
  virtual void initYourself(ActionMachine& actionMachine);
  bool finishedSuccess(ActionMachine& M);
};
