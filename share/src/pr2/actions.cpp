#include "actions.h"

// ============================================================================
// CoreTasks
CoreTasks::CoreTasks():GroundedAction("CoreTasks",0) {
}

void CoreTasks::initYourself(ActionMachine& actionMachine) {
  // PDtask *qitself;
  // qitself = P.s->MP.addPDTask("DampMotion_qitself", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, P.s->MP.H_rate_diag);
  // qitself->setGains(0.,10.);
  // qitself->y_ref = P.s->MP.qitselfPD.y_ref;
  // qitself->v_ref.setZero();
  // qitself->prec=100.;
  // tasks.append(qitself);

  PDtask* limits = actionMachine.s->MP.addPDTask("limits", .1, .8, qLimitsTMT);
  // limits->setGains(10.,0.);
  limits->v_ref.setZero();
  limits->v_ref.setZero();
  limits->prec=100.;
  tasks.append(limits);

  PDtask* coll = actionMachine.s->MP.addPDTask(
      "collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
  coll->y_ref.setZero();
  coll->v_ref.setZero();
  tasks.append(coll);
}

// ============================================================================
// MoveEffTo
MoveEffTo::MoveEffTo(const char* effName, const arr& effPos)
    : GroundedAction("MoveEffTo",2),
      effName(effName),
      effPos(effPos)
{
  SymbolL::memMove=true;
  PDtaskL::memMove=true;
}

void MoveEffTo::initYourself(ActionMachine& actionMachine) {
  PDtask *task;
  task = actionMachine.s->MP.addPDTask(
      STRING("MoveEffTo_" << effName),
      1., .8, posTMT, effName);
  // task->setGains(200.,0.);
  task->y_ref = effPos;
  tasks.append(task);
}

bool MoveEffTo::finishedSuccess(ActionMachine& M) {
  PDtask *task=tasks(0);
  return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-2);
}

// ============================================================================
// AlignEffTo
AlignEffTo::AlignEffTo(const char* effName, const arr& effPos, const arr& alignPos)
    : GroundedAction("AlignEffTo", 2),
      effName(effName),
      effPos(effPos),
      alginPos(alignPos)
{}

void AlignEffTo::initYourself(ActionMachine& actionMachine) {
  PDtask *task;
  task = actionMachine.s->MP.addPDTask(
      STRING("AlignEffTo_" << effName),
      2., .8, vecTMT, effName, ors::Vector(effPos));
  // task->setGains(100.,0.);
  task->y_ref = alginPos;
  tasks.append(task);
}

bool AlignEffTo::finishedSuccess(ActionMachine& M) {
  PDtask *task=tasks(0);
  return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
}

// ============================================================================
// PushForce
PushForce::PushForce(const char* effName, arr forceVec)
    : GroundedAction("PushForce", 2),
      effName(effName),
      forceVec(forceVec)
//    , poseArg2(poseArg2)
{}

void PushForce::initYourself(ActionMachine& actionMachine) {
  // Note that the pushtask is kinda seperate to the normal PDTasks. I does not
  // add a PDTask to the ActionMachine
}

bool PushForce::finishedSuccess(ActionMachine& M) {
  return false;
  // PDtask *task=tasks(0);
  // return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
  // return false;
}
