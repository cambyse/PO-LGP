#include "actions.h"

// ============================================================================
// CoreTasks
CoreTasks::CoreTasks(ActionMachine& actionMachine)
  : GroundedAction(actionMachine, "CoreTasks") {
  // PDtask *qitself;
  // qitself = new PDtask("DampMotion_qitself", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, P.s->MP.H_rate_diag);
  // qitself->setGains(0.,10.);
  // qitself->y_ref = P.s->MP.qitselfPD.y_ref;
  // qitself->v_ref.setZero();
  // qitself->prec=100.;
  // tasks.append(qitself);

  PDtask* limits = new PDtask("limits", .1, .8,
                              new DefaultTaskMap(qLimitsTMT, actionMachine.s->world));
  // limits->setGains(10.,0.);
  limits->v_ref.setZero();
  limits->v_ref.setZero();
  limits->prec=100.;
  tasks.append(limits);

  //PDtask* coll = new PDtask(
      //"collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
  //coll->y_ref.setZero();
  //coll->v_ref.setZero();
  //tasks.append(coll);
}

// ============================================================================
// MoveEffTo
MoveEffTo::MoveEffTo(ActionMachine& actionMachine, const char* effName, const arr& effPos)
    : GroundedAction(actionMachine, "MoveEffTo"),
      effName(effName),
      effPos(effPos)
{
  PDtaskL::memMove=true;
  PDtask *task = new PDtask(
                   STRING("MoveEffTo_" << effName), 1., .8,
                   new DefaultTaskMap(posTMT, actionMachine.s->world, effName));
  // task->setGains(200.,0.);
  task->y_ref = effPos;
  tasks.append(task);
}

bool MoveEffTo::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  PDtask *task=tasks(0);
//  return false;
  return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-2);
}

// ============================================================================
// PoseTo

PoseTo::PoseTo(ActionMachine& actionMachine, const char* effName, const arr& effPos, const arr& orientation)
    : GroundedAction(actionMachine, "PoseTo"),
      effName(effName),
      effPos(effPos),
      orientation(orientation)
{
  PDtaskL::memMove=true;
  PDtask *task = new PDtask(
                   STRING("PosTo_" << effName), 1., .8,
                   new DefaultTaskMap(posTMT, actionMachine.s->world, effName));
  task->y_ref = effPos;
  tasks.append(task);

  task = new PDtask(
           STRING("OrientatationQuat_" << effName), 1., .8,
           new DefaultTaskMap(quatTMT, actionMachine.s->world, effName, {0, 0, 0}));
  task->setTarget(orientation);
  task->flipTargetScalarProduct = true;
  tasks.append(task);
}

bool PoseTo::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  PDtask *task0=tasks(0);
  PDtask *task1=tasks(1);
  return (task0->y.N==task0->y_ref.N && maxDiff(task0->y, task0->y_ref)<1e-2) 
      && (task1->y.N == task1->y_ref.N && maxDiff(task1->y, task1->y_ref) < 1e-2);
}

// ============================================================================
// AlignEffTo
AlignEffTo::AlignEffTo(ActionMachine& actionMachine, const char* effName, const arr& effPos, const arr& alignPos)
    : GroundedAction(actionMachine, "AlignEffTo"),
      effName(effName),
      effPos(effPos),
      alginPos(alignPos)
{
  PDtask *task = new PDtask(
                   STRING("AlignEffTo_" << effName), 2., .8,
                   new DefaultTaskMap(vecTMT, actionMachine.s->world, effName, ors::Vector(effPos)));
  // task->setGains(100.,0.);
  task->y_ref = alginPos;
  tasks.append(task);
}

bool AlignEffTo::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  PDtask *task=tasks(0);
  return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
}

// ============================================================================
// OrientationQuat
OrientationQuat::OrientationQuat(ActionMachine& actionMachine, const char* effName, const arr& orientation)
    : GroundedAction(actionMachine, "OrientationQuat")
    , effName(effName)
    , orientation(orientation)
{
  PDtaskL::memMove=true;

  auto task = new PDtask(
                STRING("OrientatationQuat_" << effName), 2, .8,
                new DefaultTaskMap(quatTMT, actionMachine.s->world, effName, {0, 0, 0}));
  task->setTarget(orientation);
  task->flipTargetScalarProduct = true;
  tasks.append(task);
}

bool OrientationQuat::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  PDtask *task=tasks(0);
//  return false;
  return (task->y.N == task->y_ref.N &&
          maxDiff(task->y, task->y_ref) < .3);
}

// ============================================================================
SetQ::SetQ(ActionMachine& actionMachine, const char* effName, int jointID, double jointPos)
    : GroundedAction(actionMachine, "SetQ")
    , effName(effName)
    , jointID(jointID)
    , jointPos(jointPos)
{
  PDtaskL::memMove=true;

  auto task = new PDtask(
      effName, 2, .8, new DefaultTaskMap(qSingleTMT, jointID));
  task->setTarget({jointPos});
  task->active = true;
  tasks.append(task);
}

bool SetQ::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  PDtask *task=tasks(0);
  return false;
  return (task->y.N == task->y_ref.N &&
          maxDiff(task->y, task->y_ref) < 1e-3);
}
// ============================================================================
// PushForce
PushForce::PushForce(ActionMachine& actionMachine, const char* effName, arr forceVec)
    : GroundedAction(actionMachine, "PushForce"),
      effName(effName),
      forceVec(forceVec)
//    , poseArg2(poseArg2)
{
  // Note that the pushtask is kinda seperate to the normal PDTasks. I does not
  // add a PDTask to the ActionMachine
}

bool PushForce::finishedSuccess(ActionMachine& M) {
  return false;
  // PDtask *task=tasks(0);
  // return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
  // return false;
}
