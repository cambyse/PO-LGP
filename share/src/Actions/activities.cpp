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
//  task->active=false;
  taskController->ctrlTasks.set()->append(task);
  taskController->mutex.unlock();
  conv=false;
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


void FollowReferenceActivity::write(ostream &os) const{
  os <<"BLA" <<endl;
}
