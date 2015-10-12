#include <Motion/feedbackControl.h>
#include "TaskControllerModule.h"
#include "SensorActivities.h"

#include "taskCtrlActivities.h"

void TaskCtrlActivity::configure() {
  taskController = dynamic_cast<TaskControllerModule*>(&registry().getNode("Module","TaskControllerModule")->V<Module>());
  CHECK(taskController,"that didn't work");
  configureControl(singleString(symbols), params, taskController->modelWorld.set());
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

  stepControl(dt);

  //potentially report on stopping criteria
  mlr::String convStr = "(conv ";
  for(Node *p:fact->parents) convStr <<' ' <<p->keys.last();
  convStr <<")";
  if(isConv()){
    if(!conv){ //changed conv! -> conv
      if(fact) taskController->effects.set()() <<convStr <<", ";
      conv=true;
    }
  }else{
    if(conv){ //changed conv -> conv!
      if(fact) taskController->effects.set()() <<convStr <<"!, ";
      conv=false;
    }
  }
}

bool TaskCtrlActivity::isConv(){
  return (task->y.N && task->y.N==task->y_ref.N
      && maxDiff(task->y, task->y_ref)<stopTolerance
      && maxDiff(task->v, task->v_ref)<stopTolerance);
}

//===========================================================================
void FollowReferenceActivity::configureControl(const char *name, Graph& specs, ors::KinematicWorld& world) {
  stuck_count = 0;
  Node *it;
  if((it=specs["type"])){
    if(it->V<mlr::String>()=="wheels"){
      map = new TaskMap_qItself(world, "worldTranslationRotation");
      dynamic_cast<TaskMap_qItself*>(map)->moduloTwoPi = specs["moduloTwoPi"] ? specs["moduloTwoPi"]->V<double>() : false;
    }else if (it->V<mlr::String>()=="qItself") {
      map = new TaskMap_qItself(world.getJointByName(specs["ref1"]->V<mlr::String>())->qIndex,
                                world.getJointStateDimension());
      dynamic_cast<TaskMap_qItself*>(map)->moduloTwoPi = specs["moduloTwoPi"] ? specs["moduloTwoPi"]->V<double>() : true;
    }else{
      map = new DefaultTaskMap(specs, world);
    }
  }else{
    HALT("need a type (the map type) in the specs");
  }
  task = new CtrlTask(name, *map, specs);
  if((it=specs["tol"])) stopTolerance=it->V<double>(); else stopTolerance=1e-2;
}

void FollowReferenceActivity::stepControl(double dt){
  //if trajectory, set reference depending on actionTime
  if(ref.nd==2){
    uint t = activityTime/trajectoryDuration * (ref.d0-1);
    t = mlr::MIN(t, ref.d0-1);
    task->y_ref = ref[t];
    cout <<"STEPPING" <<endl;
  }
}

bool FollowReferenceActivity::isConv(){
  bool stuck = task->y.N == old_y.N and maxDiff(old_y, task->y) < stopTolerance;
  stuck_count = stuck ? stuck_count + 1 : 0;
  old_y = task->y;

  return ((task->y_ref.nd == 1
           && task->y.N == task->y_ref.N
           && maxDiff(task->y, task->y_ref) < stopTolerance
           && maxDiff(task->v, task->v_ref) < stopTolerance)
          or
          (task->y_ref.nd==2
           && activityTime>=trajectoryDuration)
          or (stuck and stuck_count > 6000));
}

//===========================================================================
void HomingActivity::configureControl(const char *name, Graph& specs, ors::KinematicWorld& world) {
  map = new TaskMap_qItself;
  task = new CtrlTask(name, map, 1., .8, 1., 1.);
  task->y_ref=taskController->q0;

  Node *it;
  if(&specs && (it=specs["tol"])) stopTolerance=it->V<double>(); else stopTolerance=1e-2;

  wheeljoint = world.getJointByName("worldTranslationRotation");
}

bool HomingActivity::isConv(){
  return task->y.N==task->y_ref.N
      && maxDiff(task->y, task->y_ref) < stopTolerance
      && maxDiff(task->v, task->v_ref) < stopTolerance;
}

void HomingActivity::stepControl(double dt) {
  arr b = task->y;
  if(b.N && wheeljoint && wheeljoint->qDim()){
    for(uint i=0;i<wheeljoint->qDim();i++)
      task->y_ref(wheeljoint->qIndex+i) = b(i);
  }
}

//===========================================================================
RUN_ON_INIT_BEGIN(Activities)
registerActivity<FollowReferenceActivity>("FollowReferenceActivity");
registerActivity<HomingActivity>("HomingActivity");
registerActivity<SensorActivity>("SensorActivity");
RUN_ON_INIT_END(Activities)
