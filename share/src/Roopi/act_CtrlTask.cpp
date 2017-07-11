#include "act_CtrlTask.h"
#include "roopi-private.h"
#include <Control/taskControl.h>

Act_CtrlTask::Act_CtrlTask(Roopi* r) : Act(r) {
}

Act_CtrlTask::Act_CtrlTask(Roopi *r, const Graph& specs)
  : Act_CtrlTask(r){
  TaskMap *map = TaskMap::newTaskMap(specs, roopi.getK());
  task = new CtrlTask(map->shortTag(roopi.getK()), map, specs);
  setTask(task);
  set()->active = true;
}

//Act_CtrlTask::Act_CtrlTask(Act_CtrlTask&& a)
//  : Act(&a.roopi), task(a.task), y0(a.y0), tolerance(a.tolerance){
//  a.task = NULL;
//}

Act_CtrlTask::Act_CtrlTask(Roopi *r, TaskMap* map, const arr& PD, const arr& target, const arr& prec, double tolerance)
  : Act_CtrlTask(r){
  task = new CtrlTask(map->shortTag(roopi.getK()), map);
  if(&prec && prec.N) task->prec = prec;

#if 0 //use PD reference
  //PD reference
  if(&PD && PD.N){
    task->PD().setGainsAsNatural(PD(0), PD(1));
    if(PD.N>2) task->PD().maxVel=PD(2);
    if(PD.N>3) task->PD().maxAcc=PD(3);
  }else{
    task->PD().setGainsAsNatural(1., .9);
  }
  if(&target && target.N) task->setTarget( target );
  task->PD().tolerance = tolerance;
#else //use Sine reference
  if(&PD && PD.N){
    task->setRef(new MotionProfile_Sine(target, PD(0)));
  }else{
    task->setRef(new MotionProfile_Sine(target, 2.));
  }
//  if(&target && target.N) task->PD().setTarget( target );
//  task->PD().tolerance = tolerance;
#endif

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

void Act_CtrlTask::resetStatus(){
  setStatus(AS_init);
  roopi.s->ctrlTasks.writeAccess();
  task->status=CT_init;
  roopi.s->ctrlTasks.deAccess();
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
//  if(getStatus()!=0){
//    setStatus(AS_init);
//  }
  return WToken<CtrlTask>(*roopi.s->ctrlTasks.data, task);
}

RToken<CtrlTask> Act_CtrlTask::get(){
  CHECK(task, "this is not yet configured!");
  return RToken<CtrlTask>(*roopi.s->ctrlTasks.data, task);
}

void Act_CtrlTask::write(ostream& os){ Act::write(os); if(task) os <<task->name;  }

void Act_CtrlTask::setMap(TaskMap* m){
  setTask(new CtrlTask(m->shortTag(roopi.getK()), m));
}

void Act_CtrlTask::setTask(CtrlTask *t){
  task = t;
  { //set the callback
    task->callbacks.append(
      [this](CtrlTask* t,int s){
//      if(!this->task) return;
        CHECK_EQ(t, this->task, "");
        this->setStatus(s);
      }
    );
  }
  task->update(0., roopi.getK()); //this might already call the callback
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
