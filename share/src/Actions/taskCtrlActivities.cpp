#include <Motion/feedbackControl.h>
#include "TaskControllerModule.h"

#include "taskCtrlActivities.h"

extern TaskControllerModule *taskControllerModule();

void TaskCtrlActivity::configure(Node *fact) {
  name.clear();
  for(Node *p:fact->parents) name <<p->keys.last();
  taskController = taskControllerModule();
  CHECK(taskController,"");
  Activity::fact = fact;
  Graph *specs = &NoGraph;
  if(fact->getValueType()==typeid(Graph)) specs = &fact->graph();
  configure2(name, *specs, taskController->modelWorld.set());
  taskController->ctrlTasks.set()->append(task);
  conv=false;
}

TaskCtrlActivity::~TaskCtrlActivity(){
  taskController->ctrlTasks.set()->removeValue(task);
  delete task;
  delete map;
}

void TaskCtrlActivity::step(double dt){
  activityTime += dt;

  step2(dt);

  //potentially report on stopping criteria
  MT::String convStr = "(conv ";
  for(Node *p:fact->parents) convStr <<' ' <<p->keys.last();
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
  Node *it;
  if((it=specs["type"])){
    if(it->V<MT::String>()=="wheels"){
      map = new TaskMap_qItself(world, "worldTranslationRotation");
    }else if (it->V<MT::String>()=="qItself") {
      map = new TaskMap_qItself(world.getJointByName(specs["ref1"]->V<MT::String>())->qIndex,world.getJointStateDimension());
    }else{
      map = new DefaultTaskMap(specs, world);
    }
  }else{
    HALT("need a type (the map type) in the specs");
  }
  task = new CtrlTask(name, *map, specs);
  if((it=specs["tol"])) stopTolerance=it->V<double>(); else stopTolerance=1e-2;
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
