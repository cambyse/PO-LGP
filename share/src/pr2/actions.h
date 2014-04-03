#include "actionMachine_internal.h"

/** @file This file contains implementations of GroundedActions.  */

//===========================================================================
struct CoreTasks : GroundedAction {
  CoreTasks() {
    ID=symbols().N;
    symbols().append(this);
    name="CoreTasks";
    nargs=0;
  }

  /// @name Inherited stuff
  static Symbol symbol;
  virtual Symbol& getSymbol() { return symbol; }

  virtual void initYourself(ActionMachine& actionMachine) {
//    PDtask *qitself;
//    qitself = P.s->MP.addPDTask("DampMotion_qitself", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, P.s->MP.H_rate_diag);
//    qitself->setGains(0.,10.);
//    qitself->y_ref = P.s->MP.qitselfPD.y_ref;
//    qitself->v_ref.setZero();
//    qitself->prec=100.;
//    tasks.append(qitself);

    PDtask *limits;
    limits = actionMachine.s->MP.addPDTask("limits", .1, .8, qLimitsTMT);
    // limits->setGains(10.,0.);
    limits->v_ref.setZero();
    limits->v_ref.setZero();
    limits->prec=100.;
    tasks.append(limits);
  }
};

//===========================================================================
struct MoveEffTo : GroundedAction {
  MT::String effName;
  arr effPos;

  MoveEffTo(const char* effName, const arr& effPos)
      : effName(effName)
      , effPos(effPos)
  {
    SymbolL::memMove=true;
    PDtaskL::memMove=true;
    ID=symbols().N;
    symbols().append(this);
    name="MoveEffTo";
    nargs=2;
  }

  /// @name Inherited/overwritten stuff
  static Symbol symbol;
  virtual Symbol& getSymbol() { return symbol; }

  virtual void initYourself(ActionMachine& actionMachine) {
    PDtask *task;
    task = actionMachine.s->MP.addPDTask(
        STRING("MoveEffTo_" << effName),
        1., .8, posTMT, effName);
    // task->setGains(200.,0.);
    task->y_ref = effPos;
    tasks.append(task);
  }

  virtual bool finishedSuccess(ActionMachine& M) {
    PDtask *task=tasks(0);
    return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-2);
  }
};

//===========================================================================
struct AlignEffTo : GroundedAction {
  MT::String effName;
  arr effPos;
  arr alginPos; // TODO what is this? Find a proper name.

  AlignEffTo(const char* effName, const arr& effPos, const arr& alignPos)
      : effName(effName)
      , effPos(effPos)
      , alginPos(alignPos)
  {
    ID=symbols().N;
    symbols().append(this);
    name="AlignEffTo";
    nargs=2;
  }

  /// @name Inherited stuff
  static Symbol symbol;
  virtual Symbol& getSymbol() {return symbol;}

  virtual void initYourself(ActionMachine& actionMachine) {
    PDtask *task;
    task = actionMachine.s->MP.addPDTask(
        STRING("AlignEffTo_" << effName),
        2., .8, vecTMT, effName, ors::Vector(effPos));
    // task->setGains(100.,0.);
    task->y_ref = alginPos;
    tasks.append(task);
  }

  virtual bool finishedSuccess(ActionMachine& M) {
    PDtask *task=tasks(0);
    return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
  }
};

//===========================================================================
struct PushForce : GroundedAction {
  MT::String effName;
  ors::Vector forceVec;
  arr poseArg2; // TODO what is this? Find a proper name.

  PushForce(const char* effName, ors::Vector forceVec, arr poseArg2)
      : effName(effName)
      , forceVec(forceVec)
      , poseArg2(poseArg2)
  {
    ID=symbols().N;
    symbols().append(this);
    name="PushForce";
    nargs=2;
  }

  /// @name Inherited stuff
  static Symbol symbol;
  virtual Symbol& getSymbol() {return symbol;}

  virtual void initYourself(ActionMachine& actionMachine) {
    PDtask *task;
    task = actionMachine.s->MP.addPDTask(
        STRING("PushForce_" << effName),
        .2, .8, vecTMT, effName, forceVec);
    task->y_ref = poseArg2;
    tasks.append(task);
  }
};
