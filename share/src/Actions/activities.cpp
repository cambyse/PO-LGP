#include <Motion/feedbackControl.h>
#include "activities.h"
#include <TaskControllerModule.h>

extern TaskControllerModule *taskControllerModule();

void TaskCtrlActivity::configure(Item *fact) {
  name.clear();
  for(Item *p:fact->parents) name <<p->keys.last();
  taskController = taskControllerModule();
  CHECK(taskController,"");
  Activity::fact = fact;
  Graph *specs = &NoGraph;
  if(fact->getValueType()==typeid(Graph)) specs = &fact->kvg();
  taskController->mutex.lock();
  configure2(name, *specs, taskController->modelWorld);
  taskController->ctrlTasks.set()->append(task);
  taskController->mutex.unlock();
  conv=false;
}

TaskCtrlActivity::~TaskCtrlActivity(){
  taskController->mutex.lock();
  taskController->ctrlTasks.set()->removeValue(task);
  taskController->mutex.unlock();
  delete task;
  delete map;
}

void TaskCtrlActivity::step(double dt){
  activityTime += dt;

  step2(dt);

  //potentially report on stopping criteria
  MT::String convStr = "(conv ";
  for(Item *p:fact->parents) convStr <<' ' <<p->keys.last();
  convStr <<")";
  if(isConv()){
    if(!conv){
      if(fact) taskController->effects.set()() <<convStr <<", ";
      conv=true;
    }
  }else{
    if(conv){
      if(fact) taskController->effects.set()() <<convStr <<"!, ";
      conv=false;
    }
  }
}

//===========================================================================

void FollowReferenceActivity::configure2(const char *name, Graph& specs, ors::KinematicWorld& world) {
  map = new DefaultTaskMap(specs, taskController->modelWorld);
  task = new CtrlTask(name, *map, specs);
  stopTolerance=1e-2; //TODO: overwrite from specs
}

void FollowReferenceActivity::step2(double dt){
  //if trajectory, set reference depending on actionTime
  if(ref.nd==2){
    uint t = activityTime/trajectoryDuration * (ref.d0-1);
    t = MT::MIN(t, ref.d0-1);
    task->y_ref = ref[t];
    cout <<"STEPPING" <<endl;
  }
}

bool FollowReferenceActivity::isConv(){
  return ((task->y_ref.nd==1 && task->y.N==task->y_ref.N
           && maxDiff(task->y, task->y_ref)<stopTolerance
           && maxDiff(task->v, task->v_ref)<stopTolerance)
          || (task->y_ref.nd==2 && activityTime>=trajectoryDuration));
}

//===========================================================================

void HomingActivity::configure2(const char *name, Graph& specs, ors::KinematicWorld& world) {
  map = new TaskMap_qItself;
  task = new CtrlTask(name, map, 1., .8, 1., 1.);
  task->y_ref=taskController->q0;
  stopTolerance=1e-2; //TODO: overwrite from specs
}

bool HomingActivity::isConv(){
  return task->y.N==task->y_ref.N
      && maxDiff(task->y, task->y_ref)<stopTolerance
      && maxDiff(task->v, task->v_ref)<stopTolerance;
}

//===========================================================================

RUN_ON_INIT_BEGIN(Activities)
registerActivity<FollowReferenceActivity>("FollowReferenceActivity");
registerActivity<HomingActivity>("HomingActivity");
RUN_ON_INIT_END(Activities)
