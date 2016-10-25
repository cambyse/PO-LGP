#include <Control/taskController.h>
#include <Control/TaskControllerModule.h>
#include "SensorActivities.h"
#include "ControlActivities.h"

void ControlActivity::configure() {
  taskController = getThread<TaskControllerModule>("TaskControllerModule");
  CHECK(taskController,"that didn't work");
  configureControl(singleString(symbols), params, taskController->modelWorld.set());
  taskController->ctrlTasks.set()->append(task);
  conv=false;
}

ControlActivity::~ControlActivity(){
  taskController->ctrlTasks.set()->removeValue(task);
  delete task;
  delete map;
}

void ControlActivity::activitySpinnerStep(double dt){
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

bool ControlActivity::isConv(){
  return (task->y.N && task->y.N==task->y_ref.N
      && maxDiff(task->y, task->y_ref)<stopTolerance
      && maxDiff(task->v, task->v_ref)<stopTolerance);
}

//===========================================================================

void FollowReferenceActivity::configureControl(const char *name, Graph& specs, ors::KinematicWorld& world) {
  stuck_count = 0;
  Node *it;
  if((it=specs["sym1"])){
    CHECK(!specs["type"], "can't specify type twice");
    it->keys.last()="type"; //rename sym1 to type
  }
  if((it=specs["type"])){
    if(it->get<mlr::String>()=="wheels"){
      map = new TaskMap_qItself(world, "worldTranslationRotation");
      dynamic_cast<TaskMap_qItself*>(map)->moduloTwoPi = specs["moduloTwoPi"] ? specs["moduloTwoPi"]->get<double>() : false;
    }else if (it->get<mlr::String>()=="qItself") {
      map = new TaskMap_qItself(world.getJointByName(specs["sym2"]->get<mlr::String>())->qIndex,
                                world.getJointStateDimension());
      dynamic_cast<TaskMap_qItself*>(map)->moduloTwoPi = specs["moduloTwoPi"] ? specs["moduloTwoPi"]->get<double>() : true;
    }else{
      map = new TaskMap_Default(specs, world);
    }
  }else{
    HALT("need a type (the map type) in the specs");
  }
  task = new CtrlTask(name, *map, specs);
  if((it=specs["tol"])) stopTolerance=it->get<double>(); else stopTolerance=1e-2;
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
  bool stuck = (task->y.N == old_y.N) and (maxDiff(old_y, task->y) < stopTolerance);
  stuck_count = stuck ? stuck_count + 1 : 0;
  old_y = task->y;

  return ((task->y_ref.nd == 1
           && task->y.N == task->y_ref.N
           && maxDiff(task->y, task->get_y_ref(NoArr)) < stopTolerance
           && maxDiff(task->v, task->get_ydot_ref(NoArr)) < stopTolerance)
          or (task->y_ref.nd==2 && activityTime>=trajectoryDuration)
          or (stuck and stuck_count > 6000));
}

// ============================================================================

//PushForce::PushForce(ActionMachine& actionMachine, const char* effName, arr forceVec, double _timeOut)
//    : Action(actionMachine, "controlForce") {
//  TaskMap_Default *m = new TaskMap_Default(posTMT, actionMachine.s->world, "endeffForceL");
//  CtrlTask *task = new CtrlTask(
//                     STRING("MoveEffTo_" << effName),
//                     m,
//                     1., .8, 1., 1.);
//  if(_timeOut>0.) timeOut=_timeOut;
//  task->f_ref = forceVec;
//  task->f_Igain = .003;
//  tasks.append(task);
//}

//void PushForce::step(ActionMachine& M){
//  int i=1;
//  i++;
//}

//bool PushForce::finishedSuccess(ActionMachine& M) {
//  return false;
//}

//===========================================================================

void HomingActivity::configureControl(const char *name, Graph& specs, ors::KinematicWorld& world) {
  map = new TaskMap_qItself;
  task = new CtrlTask(name, map, 1., .8, 1., 1.);
  task->y_ref=taskController->q0;

  Node *it;
  if(&specs && (it=specs["tol"])) stopTolerance=it->get<double>(); else stopTolerance=1e-2;

  wheeljoint = world.getJointByName("worldTranslationRotation");
}

bool HomingActivity::isConv(){
  return task->y.N==task->y_ref.N
      && maxDiff(task->y, task->y_ref)<stopTolerance
      && maxDiff(task->v, task->v_ref)<stopTolerance;
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
//registerActivity<FollowReferenceActivity>("Control");
//registerActivity<HomingActivity>("HomingActivity");
//registerActivity<SensorActivity>("SensorActivity");
RUN_ON_INIT_END(Activities)
