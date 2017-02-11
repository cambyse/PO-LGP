#include "act_CtrlTask.h"
#include "roopi-private.h"
#include <Control/taskControl.h>

Act_CtrlTask::Act_CtrlTask(Roopi* r) : Act(r) {
}

Act_CtrlTask::Act_CtrlTask(Roopi *r, const Graph& specs)
  : Act_CtrlTask(r){
  TaskMap *map = TaskMap::newTaskMap(specs, roopi.getKinematics());
  task = new CtrlTask(map->shortTag(roopi.getKinematics()), map, specs);
  setTask(task);
  set()->active = true;
}

Act_CtrlTask::Act_CtrlTask(Roopi *r, TaskMap* map, const arr& PD, const arr& target, const arr& prec)
  : Act_CtrlTask(r){
  task = new CtrlTask(map->shortTag(roopi.getKinematics()), map);
  if(&PD && PD.N) task->PD().setGainsAsNatural(PD(0), PD(1));
  else task->PD().setGainsAsNatural(1., .9);
  if(&PD && PD.N>2){
    task->PD().maxVel=PD(2);
    task->PD().maxAcc=PD(3);
  }
  if(&target && target.N) task->PD().y_target = target;
  if(&prec && prec.N) task->prec = prec;
  setTask(task);
  set()->active = true;
}

Act_CtrlTask::~Act_CtrlTask(){
  kill();
}

void Act_CtrlTask::start(){
  set()->active = true;
}

void Act_CtrlTask::stop(){
  set()->active = false;
}

void Act_CtrlTask::kill(){
  if(task){
    roopi.s->ctrlTasks.set()->removeValue(task);
    delete task->map;
    delete task;
    task = NULL;
  }
}

ActStatus Act_CtrlTask::getStatus(){
  CHECK(task, "this is not yet configured!")
  bool conv = false;
  roopi.s->ctrlTasks.readAccess();
  if(task->PD().isConverged(tolerance)) conv = true;
  roopi.s->ctrlTasks.deAccess();
  if(conv) return AS_converged; //status.setValue(AS_converged);
  return AS_running; //status.setValue(AS_running);
//  return (ActStatus)status.getValue();
}

WToken<CtrlTask> Act_CtrlTask::set(){
  CHECK(task, "this is not yet configured!");
  if(getValue()!=0){
    cout <<"resetting status: " <<task->name <<endl;
    setValue(AS_running);
  }
  return WToken<CtrlTask>(*roopi.s->ctrlTasks.revLock, task);
}

void Act_CtrlTask::setMap(TaskMap* m){
  setTask(new CtrlTask(m->shortTag(roopi.getKinematics()), m));
}

void Act_CtrlTask::setTask(CtrlTask *t){
  task = t;
  task->update(0., roopi.getKinematics());
  y0 = task->y;
  task->active = false;
//  task->map->phi(y0, NoArr, roopi.getKinematics()); // initialize with the current value. TODO taskControllerModule updates these only if they are active
//  task->y = y0;
  roopi.s->ctrlTasks.set()->append(task);
}

//==============================================================================


//GlobalCtrlTasks::GlobalCtrlTasks(){
//  ctrlTaskUpdater = new CtrlTaskUpdater;
//  ctrlTaskUpdater->threadLoop();
//}

//GlobalCtrlTasks::~GlobalCtrlTasks(){
//  ctrlTaskUpdater->threadClose();
//  delete ctrlTaskUpdater;
//}

RUN_ON_INIT_BEGIN(Act_CtrlTask)
mlr::Array<Act_CtrlTask*>::memMove = true;
RUN_ON_INIT_END(Act_CtrlTask)
