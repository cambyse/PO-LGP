#include "CtrlTaskAct.h"
#include "roopi-private.h"

CtrlTaskAct::CtrlTaskAct(Roopi* r, const Graph& specs)
  : roopi(r){
  map = TaskMap::newTaskMap(specs, roopi->getKinematics());
  task = new CtrlTask(map->shortTag(roopi->getKinematics()), *map, specs);
  task->active = true;
  setTask(task, false);
}

CtrlTaskAct::CtrlTaskAct(Roopi* r, TaskMap* map, const arr& PD, const arr& target, const arr& prec)
  : roopi(r), map(map){
  task = new CtrlTask(map->shortTag(roopi->getKinematics()), map);
  if(PD.N) task->setGainsAsNatural(PD(0), PD(1));
  if(PD.N>2){
    task->maxVel=PD(2);
    task->maxAcc=PD(3);
  }
  if(target.N) task->y_ref = target;
  if(prec.N) task->prec = prec;
  task->active = true;
  setTask(task, false);
}

CtrlTaskAct::~CtrlTaskAct(){
  if(task){
    roopi->s->ctrlTasks.set()->removeValue(task);
    delete task;
  }
  if(map){
    delete map;
  }
}

void CtrlTaskAct::start(){
  set()->active = true;
}

void CtrlTaskAct::stop(){
  set()->active = false;
}

ActStatus CtrlTaskAct::status(){
  CHECK(task, "this is not yet configured!")
  bool conv = false;
  roopi->s->ctrlTasks.readAccess();
  if(task->isConverged(tolerance)) conv = true;
  roopi->s->ctrlTasks.deAccess();
  if(conv) return AS_converged;
  return AS_running;
}

WToken<CtrlTask> CtrlTaskAct::set(){
  CHECK(task, "this is not yet configured!")
  return WToken<CtrlTask>(*roopi->s->ctrlTasks.revLock, task);
}

void CtrlTaskAct::setMap(TaskMap* m){
  setTask(new CtrlTask(m->shortTag(roopi->getKinematics()), m));
}

void CtrlTaskAct::setTask(CtrlTask *t, bool setDefaults){
  task = t;
  map = &task->map;
  map->phi(y0, NoArr, roopi->getKinematics()); // initialize with the current value. TODO taskControllerModule updates these only if they are active
  task->y = y0;
  if(setDefaults){
    task->y_ref = zeros(y0.N); //y0
    task->setGains(0.0,0.0);
    //  task->setC(eye(task->y_ref.d0)*1000.0); //TODO
    task->active = false;
  }
  roopi->s->ctrlTasks.set()->append(task);
}

//CtrlTaskAct& CtrlTaskAct::setGainsAsNatural(double decayTime, double dampingRatio){
//  set()->setGainsAsNatural(decayTime, dampingRatio);
//  return *this;
//}

//CtrlTaskAct& CtrlTaskAct::setGains(double kp, double kd){
//  set()->setGains(kp, kd);
//  return *this;
//}

//CtrlTaskAct& CtrlTaskAct::setReference(const arr& y_ref, bool relativeToCurrentReference){
//  if(relativeToCurrentReference) set()->y_ref += y_ref;
//  else set()->y_ref = y_ref;
//  return *this;
//}



//void Roopi::modifyCtrlTaskReference(CtrlTask* ct, const arr& yRef, const arr& yDotRef) {
//  s->ctrlTasks.writeAccess();
//  ct->setTarget(yRef, yDotRef);
//  s->ctrlTasks.deAccess();
//}

//void Roopi::modifyCtrlTaskGains(CtrlTask* ct, const arr& Kp, const arr& Kd, const double maxVel, const double maxAcc) {
//  s->ctrlTasks.writeAccess();
//  ct->setGains(Kp, Kd);
//  ct->maxVel = maxVel;
//  ct->maxAcc = maxAcc;
//  s->ctrlTasks.deAccess();
//}

//void Roopi::modifyCtrlTaskGains(CtrlTask* ct, const double& Kp, const double& Kd, const double maxVel, const double maxAcc) {
//  s->ctrlTasks.writeAccess();
//  ct->setGains(Kp, Kd);
//  ct->maxVel = maxVel;
//  ct->maxAcc = maxAcc;
//  s->ctrlTasks.deAccess();
//}

//void Roopi::modifyCtrlC(CtrlTask* ct, const arr& C) {
//  s->ctrlTasks.writeAccess();
//  ct->setC(C);
//  s->ctrlTasks.deAccess();
//}

//void Roopi::modifyForceRef(CtrlTask* ct, const arr& fRef) {
//  s->ctrlTasks.writeAccess();
//  ct->f_ref = fRef;
//  s->ctrlTasks.deAccess();
//}

//void Roopi::modifyForceAlpha(CtrlTask* ct, double fAlpha) {
//  s->ctrlTasks.writeAccess();
//  ct->f_alpha = fAlpha;
//  s->ctrlTasks.deAccess();
//}

//void Roopi::modifyForceGamma(CtrlTask* ct, double fGamma) {
//  s->ctrlTasks.writeAccess();
//  ct->f_gamma = fGamma;
//  s->ctrlTasks.deAccess();
//}

//void Roopi::modifyForce(CtrlTask* ct, const arr& fRef, const double& fAlpha, const double& fGamma) {
//  s->ctrlTasks.writeAccess();
//  if(&fRef) ct->f_ref = fRef;
//  if(&fAlpha) ct->f_alpha = fAlpha;
//  if(&fGamma) ct->f_gamma = fGamma;
//  s->ctrlTasks.deAccess();
//}
