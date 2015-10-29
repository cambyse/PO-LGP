#include "actions.h"
#include <Motion/feedbackControl.h>
#include "actionMachine_internal.h"

//===========================================================================
// Action Base Class
Action::Action(ActionMachine& actionMachine, const char* name)
    : name(name), active(false), symbol(NULL), actionTime(0.), timeOut(-1.){
  actionMachine.A.set()->append(this);
  actionMachine.KB.readAccess();
  Node *it = actionMachine.KB().getNode(name);
  if(it && &it->container != &actionMachine.KB()) it=NULL;
  if(it && it->getValueType()!=typeid(bool)) it=NULL;
  actionMachine.KB.deAccess();
  //if(!it) MLR_MSG("WARNING: there is no logic symbol for action '"<<name <<"' -- the action will be permanently deactive");
  if(it) symbol=it;
}

Action::~Action(){
//  for (CtrlTask *t : tasks) actionMachine.s->feedbackController.tasks.removeValue(t);
  listDelete(tasks);
}

void Action::reportState(ostream& os){
  os <<"Action '" <<name
    <<"':  active=" <<active
    <<"  actionTime=" <<actionTime
    <<"  CtrlTasks:" <<endl;
  for(CtrlTask* t: tasks) t->reportState(os);
  reportDetails(os);
}

// ================================================================================================
// Implementations of all Actions
// ================================================================================================

FollowReference::FollowReference(ActionMachine& actionMachine, const char* name, TaskMap *map,
                                 const arr& yref, const arr& vref, double durationInSeconds,
                                 double decayTime, double dampingRatio, double maxVel, double maxAcc,
                                 double relativePrec,
                                 double stopTolerance)
  : Action(actionMachine, name), trajectoryDuration(durationInSeconds), stopTolerance(stopTolerance)
{
  CtrlTask* task = new CtrlTask(STRING("FollowReference_" << name), map,
                                decayTime, dampingRatio, maxVel, maxAcc);
  if(yref.nd==2){
    CHECK(durationInSeconds>0., "need to specify a duration for a trajectory");
    CHECK(vref.N==0, "can't specify a vref for a trajectory");
    ref = yref;
  }else{
    CHECK(durationInSeconds<0., "can't specify a duration for a point reference");
    task->y_ref = yref;
    task->v_ref = vref;
  }
  task->prec = relativePrec;
  actionMachine.A.writeAccess();
  tasks.append(task);
  actionMachine.A.deAccess();
}

FollowReference::FollowReference(ActionMachine& actionMachine, const char* name, CtrlTask* task)
  : Action(actionMachine, name), trajectoryDuration(-1.), stopTolerance(1e-2) {
  actionMachine.A.writeAccess();
  tasks.append(task);
  actionMachine.A.deAccess();
}

void FollowReference::step(ActionMachine& M){
  if(!tasks.N) return;
  CtrlTask *task=tasks(0);
  if(task->y_ref.nd==2){
    uint t = actionTime/trajectoryDuration * (ref.d0-1);
    t = mlr::MIN(t, ref.d0-1);
    task->y_ref = ref[t];
    cout <<"STEPPING" <<endl;
  }
}

bool FollowReference::finishedSuccess(ActionMachine& M){
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
  if(task->y_ref.nd==1 && task->y.N==task->y_ref.N
     && maxDiff(task->y, task->y_ref)<stopTolerance
     && maxDiff(task->v, task->v_ref)<stopTolerance) return true;
  if(task->y_ref.nd==2 && actionTime>=trajectoryDuration) return true;
  return false;
}

void FollowReference::reportDetails(ostream& os) {
  cout <<"HELLO" <<endl;
}

// ============================================================================
// CoreTasks
CoreTasks::CoreTasks(ActionMachine& actionMachine)
    : Action(actionMachine, "CoreTasks") {
  // CtrlTask *qitself;
  // qitself = new CtrlTask("DampMotion_qitself", .1, 1., new TaskMap_qItself(P.s->MP.H_rate_diag));
  // qitself->setGains(0.,10.);
  // qitself->y_ref = P.s->MP.qitselfPD.y_ref;
  // qitself->v_ref.setZero();
  // qitself->prec=100.;
  // tasks.append(qitself);

  CtrlTask* limits = new CtrlTask("limits", new TaskMap_qLimits(), .1, .8, 1., 1.);
  // limits->setGains(10.,0.);
  limits->v_ref.setZero();
  limits->v_ref.setZero();
  limits->prec=100.;
  tasks.append(limits);

  //CtrlTask* coll = new CtrlTask(
      //"collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
  //coll->y_ref.setZero();
  //coll->v_ref.setZero();
  //tasks.append(coll);
}

//===========================================================================
Homing::Homing(ActionMachine& actionMachine, const char* name)
    : Action(actionMachine, name) {
  CtrlTask *task = new CtrlTask(
                     STRING("Homing_" << name),
                     new TaskMap_qItself(),
                     1., .8, 1., 1.);
  task->y_ref=actionMachine.s->q0;
  tasks.append(task);
}

bool Homing::finishedSuccess(ActionMachine& M){
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
  return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-2);
}

// ============================================================================
// OpenGripper
OpenGripper::OpenGripper(ActionMachine& actionMachine, const Side side)
    : Action(actionMachine, "OpenGripper") {

  const char* jointName = (side == Side::LEFT) ? "l_gripper_joint" : "r_gripper_joint";
  int jointID = actionMachine.s->world.getJointByName(jointName)->qIndex;
  cout << "joint id" << jointID << endl;
  auto task = new CtrlTask(STRING("OpenGripper_"),
                           new TaskMap_qItself(jointID, actionMachine.s->world.q.N),
                           2, .8, 1., 1.);
  task->setTarget({.08});
  // task->setGains(200, 0);
  task->active = true;
  tasks.append(task);
}

bool OpenGripper::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
  return (task->y.N == task->y_ref.N &&
          maxDiff(task->y, task->y_ref) < 1e-3);
}

// ============================================================================
// CloseGripper
CloseGripper::CloseGripper(ActionMachine& actionMachine, const Side side)
    : Action(actionMachine, "CloseGripper") {

  const char* jointName = (side == Side::LEFT) ? "l_gripper_joint" : "r_gripper_joint";
  int jointID = actionMachine.s->world.getJointByName(jointName)->qIndex;
  cout << "joint id" << jointID << endl;
  auto task = new CtrlTask(STRING("CloseGripper_"),
                           new TaskMap_qItself(jointID, actionMachine.s->world.q.N),
                           2, .8, 1., 1.);
  task->setTarget({.01});
  // task->setGains(200, 0);
  task->active = true;
  tasks.append(task);
}

bool CloseGripper::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
  return (task->y.N == task->y_ref.N &&
          maxDiff(task->y, task->y_ref) < 1e-3);
}
// ============================================================================
MoveEffTo::MoveEffTo(ActionMachine& actionMachine, const char* effName, const arr& positionTarget)
    : Action(actionMachine, "MoveEffTo") {
  CtrlTask *task = new CtrlTask(
                     STRING("MoveEffTo_" << effName),
                     new DefaultTaskMap(posTMT, actionMachine.s->world, effName),
                     1., .8, 1., 1.);
  // task->setGains(200.,0.);
  task->y_ref = positionTarget;
  tasks.append(task);
}

bool MoveEffTo::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
//  return false;
  return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-2);
}

// ============================================================================
// PoseTo

PoseTo::PoseTo(ActionMachine& actionMachine, const char* effName, const arr& positionTarget, const arr& orientationTarget)
    : Action(actionMachine, "PoseTo"){
  CtrlTask *task = new CtrlTask(
                     STRING("PosTo_" << effName),
                     new DefaultTaskMap(posTMT, actionMachine.s->world, effName),
                     1., .8, 1., 1.);
  task->y_ref = positionTarget;
  tasks.append(task);

  task = new CtrlTask(
           STRING("OrientatationQuat_" << effName),
           new DefaultTaskMap(quatTMT, actionMachine.s->world, effName, {0, 0, 0}),
            1., .8, 1., 1.);
  task->setTarget(orientationTarget);
  task->flipTargetSignOnNegScalarProduct = true;
  tasks.append(task);
}

bool PoseTo::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  CtrlTask *task0=tasks(0);
  CtrlTask *task1=tasks(1);
  return (task0->y.N==task0->y_ref.N && maxDiff(task0->y, task0->y_ref)<1e-2)
      && (task1->y.N == task1->y_ref.N && maxDiff(task1->y, task1->y_ref) < 1e-2);
}

// ============================================================================
// AlignEffTo
AlignEffTo::AlignEffTo(ActionMachine& actionMachine, const char* effName, const arr& effVector, const arr& vectorTarget)
    : Action(actionMachine, "AlignEffTo") {
  CtrlTask *task = new CtrlTask(
                     STRING("AlignEffTo_" << effName),
                     new DefaultTaskMap(vecTMT, actionMachine.s->world, effName, ors::Vector(effVector)),
                     2., .8, 1., 1.);
  // task->setGains(100.,0.);
  task->y_ref = vectorTarget;
  tasks.append(task);
}

bool AlignEffTo::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
  return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
}

// ============================================================================
// OrientationQuat
OrientationQuat::OrientationQuat(ActionMachine& actionMachine, const char* effName, const arr& orientationTarget)
    : Action(actionMachine, "OrientationQuat") {
  auto task = new CtrlTask(
                STRING("OrientatationQuat_" << effName),
                new DefaultTaskMap(quatTMT, actionMachine.s->world, effName, {0, 0, 0}),
                2, .8, 1., 1.);
  task->setTarget(orientationTarget);
  task->flipTargetSignOnNegScalarProduct = true;
  tasks.append(task);
}

bool OrientationQuat::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
//  return false;
  return (task->y.N == task->y_ref.N &&
          maxDiff(task->y, task->y_ref) < .3);
}

// ============================================================================
SetQ::SetQ(ActionMachine& actionMachine, const char* effName, int jointID, double jointPos)
    : Action(actionMachine, "SetQ") {
  auto task = new CtrlTask(
      effName, new TaskMap_qItself(jointID, actionMachine.s->world.q.N), 2, .8, 1., 1.);
  task->setTarget({jointPos});
  task->active = true;
  tasks.append(task);
}

bool SetQ::finishedSuccess(ActionMachine& M) {
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
  return false;
  return (task->y.N == task->y_ref.N &&
          maxDiff(task->y, task->y_ref) < 1e-3);
}
// ============================================================================
// PushForce
PushForce::PushForce(ActionMachine& actionMachine, const char* effName, arr forceVec, double _timeOut)
    : Action(actionMachine, "controlForce") {
  DefaultTaskMap *m = new DefaultTaskMap(posTMT, actionMachine.s->world, "endeffForceL");
  CtrlTask *task = new CtrlTask(
                     STRING("MoveEffTo_" << effName),
                     m,
                     1., .8, 1., 1.);
  if(_timeOut>0.) timeOut=_timeOut;
  task->f_ref = forceVec;
  task->f_Igain = .003;
  tasks.append(task);
}

void PushForce::step(ActionMachine& M){
  int i=1;
  i++;
}

bool PushForce::finishedSuccess(ActionMachine& M) {
  return false;
}

//===========================================================================
FollowReferenceInTaskSpace::FollowReferenceInTaskSpace(ActionMachine& actionMachine, const char* name, TaskMap *map, const arr& referenceTraj, double durationInSeconds)
  : Action(actionMachine, name), ref(referenceTraj), duration(durationInSeconds), task(NULL) {
  task = new CtrlTask(
                   STRING("FollowTraj_" << name), map, 1., .8, 1., 1.);
  // task->setGains(200.,0.);
  task->y_ref = ref[0];
//  task->v_ref = referenceTraj[0]; TODO!
  tasks.append(task);
}

void FollowReferenceInTaskSpace::step(ActionMachine& M){
  uint t = actionTime/duration * (ref.d0-1);
  t = mlr::MIN(t, ref.d0-1);
  task->y_ref = ref[t];
  cout <<"STEPPING" <<endl;
}

bool FollowReferenceInTaskSpace::finishedSuccess(ActionMachine& M){
  return actionTime>=duration;
}

void FollowReferenceInTaskSpace::reportDetails(ostream& os) {
  cout <<"HELLO" <<endl;
}

//===========================================================================

Relax::Relax(ActionMachine &actionMachine, const char *name)
  : Action(actionMachine,name) {
}

void Relax::step(ActionMachine &actionMachine) {
  actionMachine.Kp = ARR(0.);
  actionMachine.Kd = ARR(0.);
}

//===========================================================================

RUN_ON_INIT_BEGIN()
  CtrlTaskL::memMove=true;
RUN_ON_INIT_END();
