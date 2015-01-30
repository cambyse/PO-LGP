#include "actions.h"

// ============================================================================
// CoreTasks
CoreTasks::CoreTasks(ActionMachine& actionMachine)
  : GroundedAction(actionMachine, "CoreTasks") {
  // PDtask *qitself;
  // qitself = new PDtask("DampMotion_qitself", .1, 1., new TaskMap_qItself(P.s->MP.H_rate_diag));
  // qitself->setGains(0.,10.);
  // qitself->y_ref = P.s->MP.qitselfPD.y_ref;
  // qitself->v_ref.setZero();
  // qitself->prec=100.;
  // tasks.append(qitself);

  PDtask* limits = new PDtask("limits", .1, .8,
                              new TaskMap_qLimits());
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
MoveEffTo::MoveEffTo(ActionMachine& actionMachine, const char* effName, const arr& positionTarget)
    : GroundedAction(actionMachine, "MoveEffTo") {
  PDtaskL::memMove=true;
  PDtask *task = new PDtask(
                   STRING("MoveEffTo_" << effName), 1., .8,
                   new DefaultTaskMap(posTMT, actionMachine.s->world, effName));
  // task->setGains(200.,0.);
  task->y_ref = positionTarget;
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

PoseTo::PoseTo(ActionMachine& actionMachine, const char* effName, const arr& positionTarget, const arr& orientationTarget)
    : GroundedAction(actionMachine, "PoseTo"){
  PDtaskL::memMove=true;
  PDtask *task = new PDtask(
                   STRING("PosTo_" << effName), 1., .8,
                   new DefaultTaskMap(posTMT, actionMachine.s->world, effName));
  task->y_ref = positionTarget;
  tasks.append(task);

  task = new PDtask(
           STRING("OrientatationQuat_" << effName), 1., .8,
           new DefaultTaskMap(quatTMT, actionMachine.s->world, effName, {0, 0, 0}));
  task->setTarget(orientationTarget);
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
AlignEffTo::AlignEffTo(ActionMachine& actionMachine, const char* effName, const arr& effVector, const arr& vectorTarget)
    : GroundedAction(actionMachine, "AlignEffTo") {
  PDtask *task = new PDtask(
                   STRING("AlignEffTo_" << effName), 2., .8,
                   new DefaultTaskMap(vecTMT, actionMachine.s->world, effName, ors::Vector(effVector)));
  // task->setGains(100.,0.);
  task->y_ref = vectorTarget;
  tasks.append(task);
}

bool AlignEffTo::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  PDtask *task=tasks(0);
  return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
}

// ============================================================================
// OrientationQuat
OrientationQuat::OrientationQuat(ActionMachine& actionMachine, const char* effName, const arr& orientationTarget)
    : GroundedAction(actionMachine, "OrientationQuat") {
  PDtaskL::memMove=true;

  auto task = new PDtask(
                STRING("OrientatationQuat_" << effName), 2, .8,
                new DefaultTaskMap(quatTMT, actionMachine.s->world, effName, {0, 0, 0}));
  task->setTarget(orientationTarget);
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
    : GroundedAction(actionMachine, "SetQ") {
  PDtaskL::memMove=true;

  auto task = new PDtask(
      effName, 2, .8, new TaskMap_qItself(jointID, actionMachine.s->world.q.N));
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
    : GroundedAction(actionMachine, "PushForce") {
  // Note that the pushtask is kinda seperate to the normal PDTasks. I does not
  // add a PDTask to the ActionMachine
}

void PushForce::step(ActionMachine& M){
//      TODO: move to step() method of the action itself
//      if(a->name == "PushForce") {
//        cout <<" - FORCE TASK: " << endl;
//        PushForce* pf = dynamic_cast<PushForce*>(a);
//        // cout << pf->forceVec << endl;
//        s->refs.fR = pf->forceVec;
//        NIY;
////        s->refs.fR_gainFactor = 1.;
////        s->refs.Kp_gainFactor = .2;
//      }
}

bool PushForce::finishedSuccess(ActionMachine& M) {
  return false;
  // PDtask *task=tasks(0);
  // return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
  // return false;
}

//===========================================================================
FollowReferenceInTaskSpace::FollowReferenceInTaskSpace(ActionMachine& actionMachine, const char* name, TaskMap *map, const arr& referenceTraj, double durationInSeconds)
  : GroundedAction(actionMachine, name), ref(referenceTraj), duration(durationInSeconds), task(NULL) {
  PDtaskL::memMove=true;
  task = new PDtask(
                   STRING("FollowTraj_" << name), 1., .8,
                   map);
  // task->setGains(200.,0.);
  task->y_ref = ref[0];
//  task->v_ref = referenceTraj[0]; TODO!
  tasks.append(task);
}

void FollowReferenceInTaskSpace::step(ActionMachine& M){
  uint t = actionTime/duration * (ref.d0-1);
  t = MT::MIN(t, ref.d0-1);
  task->y_ref = ref[t];
  cout <<"STEPPING" <<endl;
}

bool FollowReferenceInTaskSpace::finishedSuccess(ActionMachine& M){
  return actionTime>=duration;
}

void FollowReferenceInTaskSpace::reportDetails(ostream& os) {
  cout <<"HELLO" <<endl;
}

Relax::Relax(ActionMachine &actionMachine, const char *name)
  : GroundedAction(actionMachine,name) {
}

void Relax::step(ActionMachine &actionMachine) {
  actionMachine.Kq_gainFactor = ARR(0.);
  actionMachine.Kd_gainFactor = ARR(0.);
}
