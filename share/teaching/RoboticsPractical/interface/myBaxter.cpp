#include "myBaxter.h"

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControllerModule.h>
#include <Ors/orsviewer.h>
//#include <Actions/RelationalMachineModule.h>
//#include <Actions/ActivitySpinnerModule.h>
#include <RosCom/serviceRAP.h>
#include <RosCom/baxter.h>

#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/subscribeTabletop.h>
#include "RosCom/subscribeOptitrack.h"
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

#include <baxter_core_msgs/JointCommand.h>

baxter_core_msgs::JointCommand conv_qRef2baxterMessage(const arr& q_ref, const ors::KinematicWorld& baxterModel, const char* prefix);

struct MyBaxter_private{
  Access_typed<sensor_msgs::JointState> jointState;
  ACCESSname(FilterObjects, object_database)
  ACCESSname(ors::KinematicWorld, modelWorld)

  TaskControllerModule tcm;

  RosInit rosInit;
//  SubscribeTabletop tabletop_subscriber;
//  SubscribeAlvar alvar_subscriber;
//  SubscribeOptitrack optitrack_subscriber;
  Optitrack op;
  Collector data_collector;
  Filter myFilter;
  PublishDatabase myPublisher;

  OrsViewer view;
  OrsPoseViewer ctrlView;
  SendPositionCommandsToBaxter spctb;
  Subscriber<sensor_msgs::JointState> sub;
  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  ros::NodeHandle* nh;
  ros::Publisher pub;

  MyBaxter_private()
    : jointState(NULL, "jointState"),
      tcm("baxter"),
      rosInit("MyBaxter"),
      ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld),
      spctb(tcm.realWorld),
//      data_collector(true),
      sub("/robot/joint_states", jointState)
  {
//    tcm.useRos = false;
    threadOpenModules(true);
  }

  ~MyBaxter_private(){
    delete nh;
    threadCloseModules();
  }
};


MyBaxter::MyBaxter(const bool report)
  : s(new MyBaxter_private){
    testWorld = s->tcm.realWorld;
    reportState = report;
    stopAll();
}

MyBaxter::~MyBaxter(){
  delete s;
}

CtrlTask* MyBaxter::task(const Graph& specs){
  return task("noname", specs);

}
CtrlTask* MyBaxter::task(const char* name,
                         const Graph& specs){
  TaskMap *map = TaskMap::newTaskMap(specs, s->tcm.modelWorld.get()());
  CtrlTask* t = new CtrlTask(name, *map, specs);
  map->phi(t->y, NoArr, s->tcm.modelWorld.get()()); //get the current value
  activeTasks.append(t);
  s->tcm.ctrlTasks.set() = activeTasks;

  if (reportState)
    t->reportState(cout);

  return t;
}

CtrlTask* MyBaxter::task(const char* name,
                         TaskMap* map){

  CtrlTask* t = new CtrlTask(name, map);
  map->phi(t->y, NoArr, s->tcm.modelWorld.get()()); //get the current value
  activeTasks.append(t);
  s->tcm.ctrlTasks.set() = activeTasks;

  if (reportState)
    t->reportState(cout);

  return t;
}


CtrlTask* MyBaxter::task(const char* name,
                         TaskMap* map,
                         double decayTime, double dampingRatio, double maxVel, double maxAcc){

  CtrlTask* t = new CtrlTask(name, map, decayTime, dampingRatio, maxVel, maxAcc);
  map->phi(t->y, NoArr, s->tcm.modelWorld.get()()); //get the current value
  activeTasks.append(t);
  s->tcm.ctrlTasks.set() = activeTasks;

  if (reportState)
    t->reportState(cout);

  return t;
}

CtrlTask*MyBaxter::task(const char* name,
                        TaskMap_DefaultType type,
                        const char* iShapeName, const ors::Vector& ivec,
                        const char* jShapeName, const ors::Vector& jvec,
                        const arr& target,
                        double decayTime, double dampingRatio, double maxVel, double maxAcc){
  TaskMap *map = new TaskMap_Default(type, s->tcm.modelWorld.get()(),
                                    iShapeName, ivec, jShapeName, jvec);
  CtrlTask* t = new CtrlTask(name, map, decayTime, dampingRatio, maxVel, maxAcc);
  map->phi(t->y, NoArr, s->tcm.modelWorld.get()()); //get the current value
  t->y_ref = target;
  activeTasks.append(t);
  s->tcm.ctrlTasks.set() = activeTasks;

  if (reportState)
    t->reportState(cout);

  return t;
}

CtrlTask* MyBaxter::modify(CtrlTask* t, const Graph& specs){
  s->tcm.ctrlTasks.writeAccess();
  t->set(specs);

  if (reportState)
    t->reportState(cout);

  s->tcm.ctrlTasks.deAccess();
  return t;
}

CtrlTask* MyBaxter::modifyTarget(CtrlTask* t, const arr& target){
  s->tcm.ctrlTasks.writeAccess();
  t->y_ref = target;

  if (reportState)
    t->reportState(cout);

  s->tcm.ctrlTasks.deAccess();
  return t;
}

void MyBaxter::stop(const CtrlTaskL& tasks){
  for(CtrlTask *t:tasks) { activeTasks.removeValueSafe(t); t->active = false;}
  s->tcm.ctrlTasks.set() = activeTasks;
  for(CtrlTask *t:tasks){
    delete &t->map;
    delete t;
  }
}

void MyBaxter::stopAll(){
  int count = activeTasks.N;
  cout << "In my baxter, stopall. count: " << count << endl;

  for (int i = 0; i < count; i++)
  {
    CtrlTask* t = activeTasks.first();
    cout << "removing: " << i << " of " << count << ' ' << t->name << endl;
    activeTasks.removeValueSafe(t);
    t->active = false;
    s->tcm.ctrlTasks.set() = activeTasks;
    delete &t->map;
    delete t;
  }
  cout << "Completed removal." << endl;
//  while (activeTasks.N > 0)
//  {
//    CtrlTask* t = activeTasks.first();
//    t->active = false;
//    s->tcm.ctrlTasks() = activeTasks;
//    delete &t->map;
//    delete t;
//  }
//  s->tcm.ctrlTasks.deAccess();
}

void MyBaxter::waitConv(const CtrlTaskL& tasks){
  double timeOut = 0;

  while(timeOut < 10)
  {
    mlr::wait(.03);
    timeOut += .03;
    bool allConv=true;
    for(CtrlTask *t:tasks) if(!t->isConverged()){ allConv=false; break; }
    if(allConv) return;
  }

  for (ors::Joint* joint : s->tcm.modelWorld.get()().joints)
  {
    if (joint->type != ors::JT_rigid)
    {
      cout << "Joint: " << joint->qIndex << ' ' << joint->name << endl;
    }
  }

  for(CtrlTask *t:tasks)
  {
    if(!t->isConverged())
      cout << t->name << " not converged. " << t->y_ref << " : " << t->y << endl;
    else
      cout << t->name << " converged. " << endl;
  }


  // We have spent 10 seconds getting here... do I want to just break?
  char cont = ' ';
  while (cont != 'y' && cont !='n')
  {
    cout << "Timeout getting to task. Continue waiting (y/n)? ";
    std::cin >> cont;
    cout << endl;
  }
  if (cont == 'y')
    waitConv(tasks);

  return;
}

bool MyBaxter::testConv(const CtrlTaskL& tasks, const double waitSecs){
  double timeOut = 0;
  double step = 0.03;
  while(timeOut < waitSecs)
  {
    mlr::wait(step);
    timeOut += step;
    bool allConv=true;
    for(CtrlTask *t:tasks) if(!t->isConverged()){ allConv=false; break; }
    if(allConv) return true;
  }

  return false;
}

bool MyBaxter::testRealConv(const CtrlTaskL& tasks, const double waitSecs){
  double timeOut = 0;
  double step = 0.03;
  while(timeOut < waitSecs)
  {
    mlr::wait(step);
    timeOut += step;
    bool allConv=true;
    for(CtrlTask *t:tasks)
    {
      arr y;
      t->map.phi(y, NoArr, s->tcm.realWorld);

      bool converged = (y.N && y.N==t->y_ref.N && t->v.N==t->v_ref.N
                       && maxDiff(y, t->y_ref)< 0.01
                       && maxDiff(t->v, t->v_ref)< 0.01);

      if (timeOut > (waitSecs - 0.05))
      {
        cout << t->name << " converged: " << converged << endl;
        cout << y << endl;
        cout << t->y_ref << endl;
      }

      if(!converged)
      {
        allConv=false; break;
      }
    }
    if(allConv) return true;
  }

  return false;
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
  if(s->nh){
    sensor_msgs::JointState js = s->jointState.get();

    std::cout << "Joint header: " << js.header.seq << std::endl;
    for (uint i = 0; i < js.name.size(); ++i){
      std::cout << "\tJoint: " << js.name[i] << "\tPosition: " << js.position[i] << "\tEffort: " << js.effort[i] << std::endl;
    }
  }
}

arr MyBaxter::getEfforts(){
  arr u;
  for(;;){
    u = baxter_getEfforts(s->jointState.get(), s->tcm.realWorld);
    if(fabs(u(0))>1e-10) return u;
    s->jointState.waitForNextRevision();
  }
  return u;
}

void MyBaxter::getState(arr& q, arr& qdot, arr& u){
  for(;;){
    baxter_get_q_qdot_u(q, qdot, u, s->jointState.get(), s->tcm.realWorld);
    if(fabs(q(0))>1e-10) return;
    s->jointState.waitForNextRevision();
  }
}


double MyBaxter::setTestJointState(const arr &q){
  testWorld.setJointState(q);
  arr y;
  testWorld.kinematicsProxyCost(y, NoArr);
  testWorld.gl("testWorld").update();
  return y.scalar();
}

double MyBaxter::updateLockbox(const ors::Transformation& tf){
  ///////////////////
  s->tcm.realWorld.getBodyByName("lockbox")->X = tf;
  for (auto shape : s->tcm.realWorld.getBodyByName("lockbox")->shapes)
    shape->X = tf;

  s->tcm.realWorld.calc_fwdPropagateFrames();
  s->tcm.realWorld.calc_fwdPropagateShapeFrames();

  ///////////////////
  s->modelWorld.set()->getBodyByName("lockbox")->X = tf;
  for (auto shape : s->modelWorld.set()->getBodyByName("lockbox")->shapes)
    shape->X = tf;

  s->modelWorld.set()->calc_fwdPropagateFrames();
  s->modelWorld.set()->calc_fwdPropagateShapeFrames();

  ///////////////////
  s->tcm.modelWorld.set()->getBodyByName("lockbox")->X = tf;
  for (auto shape : s->tcm.modelWorld.set()->getBodyByName("lockbox")->shapes)
    shape->X = tf;

  s->tcm.modelWorld.set()->calc_fwdPropagateFrames();
  s->tcm.modelWorld.set()->calc_fwdPropagateShapeFrames();

  for (auto kw : s->ctrlView.copies)
  {
    kw->getBodyByName("lockbox")->X = tf;
    for (auto shape : kw->getBodyByName("lockbox")->shapes)
      shape->X = tf;

    kw->calc_fwdPropagateFrames();
    kw->calc_fwdPropagateShapeFrames();
  }
//  s->ctrlView.copies(0)->calc_fwdPropagateShapeFrames();
 // s->tcm.realWorld.calc_fwdPropagateShapeFrames();

//  s->modelWorld.set()->getBodyByName("lockbox")->X = tf;

//        arr qdot = s->modelWorld.get()->qdot;

//        s->tcm.realWorld.setJointState(q, qdot);
//        s->tcm.modelWorld.set()->setJointState(q, qdot);



//        s->tcm.ctrl_q_real.set() = q;
//        s->tcm.ctrl_q_ref.set() = q;
//        s->tcm.q_real = q;
//        s->tcm.q_model = q;
//        s->tcm.q0 = q;

//        s->tcm.modelWorld.set()->gl("testWindow").update();
//  arr qdot = s->modelWorld.get()->qdot;
//  s->tcm.realWorld.q = q;
//  s->tcm.realWorld.gl("modelWorld").update();
  arr y;
  s->tcm.realWorld.kinematicsProxyCost(y, NoArr);
  return y.scalar();
}


void MyBaxter::setRealWorld(arr& q)
{
//  cout << "Before: " << s->tcm.realWorld.q << endl;
  s->tcm.realWorld.setJointState(q);
  s->tcm.ctrl_q_real.set() = q;

}

void MyBaxter::getEquationOfMotion(arr& M, arr& F){
  testWorld.equationOfMotion(M, F);
}

arr MyBaxter::getJointState(){
  return s->tcm.realWorld.q;
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

ors::Vector MyBaxter::arPose(){
  s->object_database.readAccess();

  ors::Vector toReturn(0,0,0);

  for(FilterObject* fo : s->object_database())
  {
    if ((fo->id == 2) && (fo->type == FilterObject::FilterObjectType::alvar))
    {
      ors::Transformation pos = fo->frame * fo->transform;
      toReturn = pos.pos;
      std::cout << toReturn << std::endl;
    }
  }
  s->object_database.deAccess();

  return toReturn;
}


void MyBaxter::publishTorque(const arr& u, const char* prefix){
  if(s->nh){
    //cout <<"SENDING TORQUES: " <<u <<endl;
    baxter_core_msgs::JointCommand msg = conv_qRef2baxterMessage(u, s->tcm.realWorld, prefix);
    msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
    s->pub.publish(msg);
  }
}

const ors::KinematicWorld& MyBaxter::getKinematicWorld(){
  return s->tcm.realWorld;
}

const ors::KinematicWorld& MyBaxter::getModelWorld(){
  return s->tcm.modelWorld.get();
}

arr MyBaxter::getJointLimits(){
  return s->tcm.realWorld.getLimits();
}

void MyBaxter::setLimits(){
  arr limits = s->tcm.realWorld.getLimits();

  TaskMap *map = new TaskMap_qLimits(limits);

  CtrlTask* t = new CtrlTask("limits", map);

  t->map.phi(t->y, NoArr, s->tcm.modelWorld.get()()); //get the current value

  activeTasks.append(t);
  s->tcm.ctrlTasks.set() = activeTasks;

  if (reportState)
    t->reportState(cout);

}

void MyBaxter::releaseLimits(){
  for (CtrlTask* t : activeTasks)
  {
    if (t->name == "limits")
    {
      t->active = false;
      activeTasks.removeValue(t);
      s->tcm.ctrlTasks.set() = activeTasks;
      delete &t->map;
      delete t;
      return;
    }
  }
}


double MyBaxter::getCollisionScalar(){
  arr y;
  s->tcm.modelWorld.get()->kinematicsProxyCost(y, NoArr);
  return y.scalar();
}

TaskControllerModule& MyBaxter::getTaskControllerModule(){
  return s->tcm;
}

void MyBaxter::disablePosControl(){
  s->spctb.enablePositionControlL = false;
}

void MyBaxter::enablePosControl(){
  s->spctb.enablePositionControlL = true;
}

void MyBaxter::enableTotalTorqueMode(){
  s->spctb.totalTorqueModeR = true;
}

void MyBaxter::disableTotalTorqueMode(){
  s->spctb.totalTorqueModeR = false;
}

void MyBaxter::grip(){
  grip(!isGripping);
}

void MyBaxter::grip(const bool toGrip, const bool sim){
  NIY;
  arr q = s->tcm.modelWorld.get()->q;
  ors::Joint *j = s->tcm.modelWorld.get()->getJointByName("l_gripper_l_finger_joint");
  isGripping = toGrip;
  toGrip ? q(j->qIndex) = 0 : q(j->qIndex) = 1;

  std::cout << "Gripping: " << isGripping << std::endl;

  if (gripTask)
    stop({gripTask});

  gripTask = task(GRAPH("map=qItself PD=[1., 1, 3., 2.] prec=[100.]"));
  modifyTarget(gripTask, q);

  uint count = 0;
  arr pos;
  do
  {
    count++;
    if (count > 500)
      break;
    mlr::wait(0.01);
//    if (sim)
      pos = s->tcm.modelWorld.get()->q;
  //  else
    //  pos = s->tcm.realWorld.q;

  } while( std::abs(50 * pos(j->qIndex) - q(j->qIndex)) > 0.1);
//  stop({grip});
}

//RelationalMachineModule& MyBaxter::rm(){
//  return s->rm;
//}
