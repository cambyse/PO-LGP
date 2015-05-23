#include <Motion/feedbackControl.h>
#include "activities.h"
#include <TaskControllerModule.h>

extern TaskControllerModule *taskControllerModule();

void FollowReferenceActivity::configure(Item *fact) {
  taskController = taskControllerModule();
  CHECK(taskController,"");
  Activity::fact = fact;
  Graph &specs = fact->kvg();
  taskController->mutex.lock();
  map = new DefaultTaskMap(specs, taskController->modelWorld);
  task = new CtrlTask(fact->parents(0)->keys.last(), *map, specs);
  stopTolerance=1e-2; //TODO: overwrite from specs
  taskController->ctrlTasks.set()->append(task);
  taskController->mutex.unlock();
  conv=false;
}

FollowReferenceActivity::~FollowReferenceActivity(){
  taskController->mutex.lock();
  taskController->ctrlTasks.set()->removeValue(task);
  taskController->mutex.unlock();
  delete task;
  delete map;
}

void FollowReferenceActivity::step(RelationalMachine& RM, double dt){
  //if trajectory, set reference depending on actionTime
  if(ref.nd==2){
    uint t = actionTime/trajectoryDuration * (ref.d0-1);
    t = MT::MIN(t, ref.d0-1);
    task->y_ref = ref[t];
    cout <<"STEPPING" <<endl;
  }

  //potentially report on stopping criteria
  if((task->y_ref.nd==1 && task->y.N==task->y_ref.N
      && maxDiff(task->y, task->y_ref)<stopTolerance
      && maxDiff(task->v, task->v_ref)<stopTolerance)
     || (task->y_ref.nd==2 && actionTime>=trajectoryDuration)){
    if(!conv){
      if(fact) taskController->effects.set()() <<"(conv " <<fact->parents(0)->keys.last() <<"), ";
      conv=true;
    }
  }else{
    if(conv){
      if(fact) taskController->effects.set()() <<"(conv " <<fact->parents(0)->keys.last() <<")!, ";
      conv=false;
    }
  }
}


void FollowReferenceActivity::write(ostream &os) const{
  os <<"FollowReference";
}

//===========================================================================

void HomingActivity::configure(Item *fact) {
  taskController = taskControllerModule();
  CHECK(taskController,"");
  Activity::fact = fact;
  map = new TaskMap_qItself;
  task = new CtrlTask("Homing",
                      map,
                      1., .8, 1., 1.);
  taskController->mutex.lock();
  task->y_ref=taskController->q0;
  taskController->ctrlTasks.set()->append(task);
  taskController->mutex.unlock();
  conv=false;
}

HomingActivity::~HomingActivity(){
  taskController->mutex.lock();
  taskController->ctrlTasks.set()->removeValue(task);
  taskController->mutex.unlock();
  delete task;
  delete map;
}

void HomingActivity::step(RelationalMachine& RM, double dt){
  //potentially report on stopping criteria
  if(task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-2){
    if(!conv){
      taskController->effects.set()() <<"(conv " <<fact->parents(0)->keys.last() <<"), ";
      conv=true;
    }
  }else{
    if(conv){
      taskController->effects.set()() <<"(conv " <<fact->parents(0)->keys.last() <<")!, ";
      conv=false;
    }
  }
}

void HomingActivity::write(ostream &os) const{
  os <<"Homing";
}

