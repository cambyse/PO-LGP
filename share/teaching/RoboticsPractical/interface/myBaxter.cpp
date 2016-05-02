#define REPORT 1

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

#include <RosCom/subscribeTabletop.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

struct MyBaxter_private{
  Access_typed<sensor_msgs::JointState> jointState;
  ACCESSname(FilterObjects, object_database)

  TaskControllerModule tcm;
//  RelationalMachineModule rm;
//  ActivitySpinnerModule aspin;

  RosInit rosInit;
  SubscribeTabletop tabletop_subscriber;
  Collector data_collector;
  Filter myFilter;
  PublishDatabase myPublisher;

  GamepadInterface gamepad;
  OrsViewer view;
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
#ifdef REPORT
  t->reportState(cout);
#endif
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
#ifdef REPORT
  t->reportState(cout);
#endif
  return t;
}

CtrlTask* MyBaxter::modify(CtrlTask* t, const Graph& specs){
  s->tcm.ctrlTasks.writeAccess();
  t->set(specs);
#ifdef REPORT
  t->reportState(cout);
#endif
  s->tcm.ctrlTasks.deAccess();
  return t;
}

CtrlTask* MyBaxter::modifyTarget(CtrlTask* t, const arr& target){
  s->tcm.ctrlTasks.writeAccess();
  t->y_ref = target;

#ifdef REPORT
  t->reportState(cout);
#endif
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

uint MyBaxter::reportPerceptionObjects(){
  s->object_database.readAccess();
  FilterObjects clusters;
  uint n=0;
  for(FilterObject* fo : s->object_database()){
    fo->write(cout);
    cout <<endl;
    n++;
  }
  s->object_database.deAccess();
  return n;
}

void MyBaxter::reportJointState(){
  sensor_msgs::JointState js = s->jointState.get();

  std::cout << "Joint header: " << js.header.seq << std::endl;
  for (uint i = 0; i < js.name.size(); ++i){
    std::cout << "\tJoint: " << js.name[i] << "\tPosition: " << js.position[i] << "\tEffort: " << js.effort[i] << std::endl;
  }

}

ors::Vector MyBaxter::closestCluster(){
  s->object_database.readAccess();

  ors::Vector toReturn(0,0,0);

  double max_dist = DBL_MIN;
  for(FilterObject* fo : s->object_database())
  {
    if (fo->type == FilterObject::FilterObjectType::cluster)
    {
      ors::Vector mean = dynamic_cast<Cluster*>(fo)->transform.pos;
      double dist = dynamic_cast<Cluster*>(fo)->transform.pos.z;
      if (max_dist < dist)
      {
        max_dist = dist;
        toReturn = mean;
      }
    }
  }
  s->object_database.deAccess();

  return toReturn;
}

arr MyBaxter::q0(){
  return s->tcm.q0;
}

//RelationalMachineModule& MyBaxter::rm(){
//  return s->rm;
//}
