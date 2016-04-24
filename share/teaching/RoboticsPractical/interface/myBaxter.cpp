#include "myBaxter.h"

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
//#include <Actions/RelationalMachineModule.h>
//#include <Actions/ActivitySpinnerModule.h>
#include <RosCom/serviceRAP.h>
#include <RosCom/baxter.h>

struct MyBaxter_private{
  Access_typed<sensor_msgs::JointState> jointState;

  TaskControllerModule tcm;
//  RelationalMachineModule rm;
//  ActivitySpinnerModule aspin;

  RosInit rosInit;
  GamepadInterface gamepad;
  OrsPoseViewer ctrlView;
  SendPositionCommandsToBaxter spctb;
  Subscriber<sensor_msgs::JointState> sub;
//  ServiceRAP rapservice;
  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  MyBaxter_private()
    : jointState(NULL, "jointState"),
      tcm("baxter"),
      rosInit("MyBaxter"),
      ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld),
      sub("/robot/joint_states", jointState)
  {
    //-- ugly...
//    for(Node *n:registry().getNodes("Activity")) rm.newSymbol(n->keys.last().p);
//    for(ors::Shape *sh:tcm.realWorld.shapes) rm.newSymbol(sh->name.p);

    threadOpenModules(true);
  }

  ~MyBaxter_private(){
    threadCloseModules();
  }
};


MyBaxter::MyBaxter()
  : s(new MyBaxter_private){
}

MyBaxter::~MyBaxter(){
  delete s;
}

CtrlTask* MyBaxter::task(const Graph& specs){
  TaskMap *map = TaskMap::newTaskMap(specs, s->tcm.modelWorld.get()());
  CtrlTask* t = new CtrlTask("noname", *map, specs);
  map->phi(t->y, NoArr, s->tcm.modelWorld.get()()); //get the current value
  activeTasks.append(t);
  s->tcm.ctrlTasks.set() = activeTasks;
  return t;
}

CtrlTask*MyBaxter::task(const char* name,
                        DefaultTaskMapType type,
                        const char* iShapeName, const ors::Vector& ivec,
                        const char* jShapeName, const ors::Vector& jvec,
                        const arr& target,
                        double decayTime, double dampingRatio, double maxVel, double maxAcc){
  TaskMap *map = new DefaultTaskMap(type, s->tcm.modelWorld.get()(),
                                    iShapeName, ivec, jShapeName, jvec);
  CtrlTask* t = new CtrlTask(name, map, decayTime, dampingRatio, maxVel, maxAcc);
  map->phi(t->y, NoArr, s->tcm.modelWorld.get()()); //get the current value
  t->y_ref = target;
  activeTasks.append(t);
  s->tcm.ctrlTasks.set() = activeTasks;
  return t;
}

CtrlTask* MyBaxter::modify(CtrlTask* t, const Graph& specs){
  s->tcm.ctrlTasks.writeAccess();
  t->set(specs);
  s->tcm.ctrlTasks.deAccess();
  return t;
}

CtrlTask* MyBaxter::modifyTarget(CtrlTask* t, const arr& target){
  s->tcm.ctrlTasks.writeAccess();
  t->y_ref = target;
  s->tcm.ctrlTasks.deAccess();
  return t;
}

void MyBaxter::stop(const CtrlTaskL& tasks){
  for(CtrlTask *t:tasks) activeTasks.removeValue(t);
  s->tcm.ctrlTasks.set() = activeTasks;
  for(CtrlTask *t:tasks){
    delete &t->map;
    delete t;
  }
}

void MyBaxter::waitConv(const CtrlTaskL& tasks){
  for(;;){
    mlr::wait(.03);
    bool allConv=true;
    for(CtrlTask *t:tasks) if(!t->isConverged()){ allConv=false; break; }
    if(allConv) return;
  }
}

arr MyBaxter::q0(){
  return s->tcm.q0;
}

//RelationalMachineModule& MyBaxter::rm(){
//  return s->rm;
//}
