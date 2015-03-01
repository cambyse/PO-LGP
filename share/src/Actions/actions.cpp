#include "actions.h"
#include <Motion/feedbackControl.h>
#include "actionMachine_internal.h"

//===========================================================================
// Action
//

Action::Action(ActionMachine& actionMachine, const char* name, ActionState actionState)
  : name(name), actionState(actionState), symbol(NULL), actionTime(0.){
  actionMachine.A.set()->append(this);
  actionMachine.KB.readAccess();
  Item *it = actionMachine.KB().getItem(name);
  if(it && &it->container != &actionMachine.KB()) it=NULL;
  if(it && it->getValueType()!=typeid(bool)) it=NULL;
  actionMachine.KB.deAccess();
  if(!it) MT_MSG("WARNING: there is no logic symbol for action '"<<name <<"' -- the action will be permanently deactive");
  if(it) symbol=it;
}

Action::~Action(){
//  for (CtrlTask *t : tasks) actionMachine.s->feedbackController.tasks.removeValue(t);
  listDelete(tasks);
}

void Action::reportState(ostream& os){
  os <<"Action '" <<name
    <<"':  actionState=" << getActionStateString(actionState)
    <<"  actionTime=" << actionTime
    <<"  CtrlTasks:" <<endl;
  for(CtrlTask* t: tasks) t->reportState(os);
  reportDetails(os);
}

// ============================================================================
FollowReference::FollowReference(ActionMachine& actionMachine, const char* name, TaskMap *map,
    const arr& yref, const arr& vref, double durationInSeconds,
    double decayTime, double dampingRatio, double maxVel, double maxAcc,
    double relativePrec,
    double stopTolerance, bool stopOnContact)
  : Action(actionMachine, name), duration(durationInSeconds), stopTolerance(stopTolerance), stopOnContact(stopOnContact) {
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
  : Action(actionMachine, name), duration(-1.), stopTolerance(1e-2), stopOnContact(true) {
  actionMachine.A.writeAccess();
  tasks.append(task);
  actionMachine.A.deAccess();
}

void FollowReference::step(ActionMachine& M){
  if(!tasks.N) return;
  CtrlTask *task=tasks(0);
  if(task->y_ref.nd==2){
    uint t = actionTime/duration * (ref.d0-1);
    t = MT::MIN(t, ref.d0-1);
    task->y_ref = ref[t];
    cout <<"STEPPING" <<endl;
  }
}

bool FollowReference::finishedSuccess(ActionMachine& M){
  if(!tasks.N) return false;
  CtrlTask *task=tasks(0);
  if(stopOnContact){
    arr fL = M.ctrl_obs.get()->fL;
    if(absMax(fL)>2.) return true;
  }
  if(task->y_ref.nd==1 && task->y.N==task->y_ref.N
     && maxDiff(task->y, task->y_ref)<stopTolerance
     && maxDiff(task->v, task->v_ref)<stopTolerance) return true;
  if(task->y_ref.nd==2 && actionTime>=duration) return true;
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
PushForce::PushForce(ActionMachine& actionMachine, const char* effName, arr forceVec)
    : Action(actionMachine, "PushForce") {
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
  // CtrlTask *task=tasks(0);
  // return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
  // return false;
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

//===========================================================================

Relax::Relax(ActionMachine &actionMachine, const char *name)
  : Action(actionMachine,name) {
}

void Relax::step(ActionMachine &actionMachine) {
  actionMachine.Kq_gainFactor = ARR(0.);
  actionMachine.Kd_gainFactor = ARR(0.);
}

//===========================================================================

struct RUN_ON_INIT{
  RUN_ON_INIT(){
    CtrlTaskL::memMove=true;
  }
} dummy;
