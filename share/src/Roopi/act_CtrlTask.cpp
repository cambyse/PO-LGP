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

Act_CtrlTask::Act_CtrlTask(Act_CtrlTask&& a)
  : Act(&a.roopi), task(a.task), y0(a.y0), tolerance(a.tolerance){
  a.task = NULL;
}

Act_CtrlTask::Act_CtrlTask(Roopi *r, TaskMap* map, const arr& PD, const arr& target, const arr& prec, double tolerance)
  : Act_CtrlTask(r){
  task = new CtrlTask(map->shortTag(roopi.getKinematics()), map);
  if(&prec && prec.N) task->prec = prec;

  //PD reference
  if(&PD && PD.N) task->PD().setGainsAsNatural(PD(0), PD(1));
  else task->PD().setGainsAsNatural(1., .9);
  if(&PD && PD.N>2){
    task->PD().maxVel=PD(2);
    task->PD().maxAcc=PD(3);
  }
  if(&target && target.N) task->PD().setTarget( target );
  task->PD().tolerance = tolerance;

  setTask(task);
  set()->active = true;
}

Act_CtrlTask::~Act_CtrlTask(){
  if(task){
    roopi.s->ctrlTasks.set()->removeValue(task);
    delete task;
    task = NULL;
  }
}

void Act_CtrlTask::start(){
  set()->active = true;
}

void Act_CtrlTask::stop(){
  set()->active = false;
}

//void Act_CtrlTask::kill(){
//}


WToken<CtrlTask> Act_CtrlTask::set(){
  CHECK(task, "this is not yet configured!");
  if(getStatus()!=0){
    setStatus(AS_running);
  }
  return WToken<CtrlTask>(*roopi.s->ctrlTasks.data, task);
}

RToken<CtrlTask> Act_CtrlTask::get(){
  CHECK(task, "this is not yet configured!");
  return RToken<CtrlTask>(*roopi.s->ctrlTasks.data, task);
}

void Act_CtrlTask::setMap(TaskMap* m){
  setTask(new CtrlTask(m->shortTag(roopi.getKinematics()), m));
}

void Act_CtrlTask::setTask(CtrlTask *t){
  task = t;
  task->update(0., roopi.getKinematics());
  y0 = task->y;
  task->active = false;
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
