#include "roopi.h"
#include "roopi-private.h"

#include <Gui/viewer.h>
#include <Kin/PhysXThread.h>
#include <Kin/kin_swift.h>
#include <Kin/frame.h>
#include <Kin/switch.h>
#include <Control/GamepadControlThread.h>
#include <RosCom/spinner.h>

#define baxter 0

//==============================================================================

Roopi_private::Roopi_private(Roopi* roopi)
  : modelWorld(NULL, "modelWorld"),
    ctrlTasks(NULL, "ctrlTasks"){
}

Roopi_private::~Roopi_private(){
  threadReportCycleTimes();

  //delete persistant acts
  _tweets.reset();
  _ComRos.reset();//these need to be done by hand, in this order!
  _ComPR2.reset();
//  if(_tweets) delete _tweets; _tweets=NULL; //somehow the tweeter crashes to tweet the other's kill...
//  if(_ComRos) delete _ComRos; _ComRos=NULL; //shut of the spinner BEFORE you close the pubs/subscribers..
//  if(_ComPR2) delete _ComPR2; _ComPR2=NULL;
//  if(_holdPositionTask) delete _holdPositionTask; _holdPositionTask=NULL;
//  if(_watchTask) delete _watchTask; _watchTask=NULL;
//  if(_collTask) delete _collTask; _collTask=NULL;
//  if(_taskController) delete _taskController; _taskController=NULL;
//  if(_taskUpdater) delete _taskUpdater; _taskUpdater=NULL;
//  if(_ctrlView) delete _ctrlView; _ctrlView=NULL;
  //threadCloseModules();
  cout << "bye bye" << endl;
}

//==============================================================================

Roopi::Roopi(bool autoStartup, bool controlView)
  : s(new Roopi_private(this)), acts(NULL, "acts"){
  s->model = mlr::getParameter<mlr::String>("model", "model.g");
  s->robot = mlr::getParameter<mlr::String>("robot", "pr2");
  s->useRos = mlr::getParameter<bool>("useRos", false);
  s->hyperSpeed = mlr::getParameter<double>("hyperSpeed", -1.);

  if(autoStartup){
    if(s->useRos){
      s->_ComRos = newRosSpinner();
      s->_ComPR2 = newComPR2();
    }

    startTweets();
    setKinematics(s->model, controlView);
    startTaskController();
  }
}

Roopi::~Roopi(){
  delete s;
}

void Roopi::report(){
  cout <<"----ROOPI REPORT------" <<endl;
  acts.readAccess();
  for(Act* a:acts()){ a->write(cout); cout <<endl; }
  acts.deAccess();
  cout <<"----------------------" <<endl;
}

//==============================================================================
//
// basic flow control
//

int _allPositive(const SignalerL& signalers, const intA& statuses, int whoChanged){
  bool allPositive=true;
  for(const int& s:statuses) if(s<=0) allPositive=false;
  if(allPositive) return AS_true;
  return AS_false;
}

bool Roopi::wait(const SignalerL& acts, double timeout){
#if 0
  double startTime = mlr::realTime();
  for(;;){
    bool allConv = true;
    for(Act *act : acts) if(act->getStatus()<=0){ allConv=false; break; }
    if(allConv) return true;

    if(timeout>0 && mlr::realTime() - startTime > timeout) {
      cout << "not converged, timeout reached" << endl;
      return false;
    }

    mlr::wait(0.1);
  }
  return false;
#elif 0
  double startTime = mlr::realTime();
  Signaler waiter;
  for(Signaler* act : acts) waiter.listenTo(*act);
  for(;;){
    bool allConv = true;
    for(Signaler* act : acts) if(act->getStatus()<=0) allConv=false;
    if(allConv) return true;

    waiter.statusLock();
    // The typical pattern (also in Thread): CondVar status==0 -> idle; CondVar status>0 -> there is work to do (and 'messengers' lists who set it to 1)
    if(waiter.status==0){ //no new signals
      if(timeout>0.){
        if(mlr::realTime()-startTime > timeout){ waiter.mutex.unlock(); return false; }
        if(!waiter.waitForSignal(timeout, true)){
          LOG(0) <<"timeout reached";
          waiter.statusUnlock(); return false;
        }
      }else{
        waiter.waitForSignal(true);
      }
    }
    waiter.status=0;
    waiter.statusUnlock();

  }
  return true;
#else
  return waitEvent(event(acts, _allPositive), timeout);
#endif
}

bool Roopi::wait(){
  return mlr::wait(true);
}

void Roopi::wait(double seconds){
  if(!s->useRos && s->hyperSpeed>0.) mlr::wait(seconds/s->hyperSpeed);
  else mlr::wait(seconds);
}

bool Roopi::waitEvent(const Act_Event::Ptr& event, double timeout){
  return event->waitForStatusEq(AS_true, false, timeout);
}

Act::Ptr Roopi::event(SignalerL sigs, const EventFunction& eventFct){
  return make_shared<Act_Event>(this, sigs, eventFct);
}

Act::Ptr Roopi::at(const Act_Event::Ptr& event, const std::function<int()>& script){
  return make_shared<Act_AtEvent>(this, event, script);
}

Act::Ptr Roopi::run(const std::function<int ()>& script){
  return make_shared<Act_Script>(this, script);
}

Act::Ptr Roopi::loop(double beatIntervalSec, const std::function<int ()>& script){
  return make_shared<Act_Script>(this, script, beatIntervalSec);

}


VariableBase::Ptr Roopi::variableStatus(const char* var_name){
  return getVariable(var_name);
//  Node *n = registry()->findNode({"VariableData", var_name});
//  if(n){
//    VariableBase::Ptr v = n->getPtr<VariableBase>();
//    if(v) return v;
//  }
//  LOG(-1) <<"Variable '" <<var_name <<"' does not exist!";
//  return NULL;
}

//==============================================================================
//
// robotics stuff
//

void Roopi::setKinematics(const char* filename, bool controlView){
  mlr::String name(filename);
  mlr::KinematicWorld K;
  if(name=="pr2") {
    K.init(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  } else if(name=="baxter") {
    K.init(mlr::mlrPath("data/baxter_model/baxter.ors").p);
  } else {
    K.init(name);
  }

  setKinematics(K, controlView);
}

void Roopi::setKinematics(const mlr::KinematicWorld& K, bool controlView){
  s->modelWorld.set() = K;
  s->q0 = K.q;

  if(controlView){
//    if(s->useRos){
//      s->_ctrlView = shared_ptr<Act_Thread>(new Act_Thread(this, new OrsPoseViewer("modelWorld", {"ctrl_q_ref", "ctrl_q_real"}, .1)));
//    } else {
    s->_ctrlView = make_shared<Act_Thread>(this, new OrsViewer("modelWorld", .1));
//    }
  }
}

shared_ptr<Act_TaskController> Roopi::startTaskController(){
  s->_taskController = make_shared<Act_TaskController>(this);
  return s->_taskController;
}

Act::Ptr Roopi::startTweets(bool go){
  if(!s->_tweets && go) s->_tweets = make_shared<Act_Tweets>(this);
  if(s->_tweets && !go) s->_tweets.reset();
  return s->_tweets;
}

Act_TaskController& Roopi::getTaskController(){
  return *s->_taskController;
}

Act_ComPR2& Roopi::getComPR2(){
  return *s->_ComPR2;
}

Act::Ptr Roopi::rosPublish(const VariableBase::Ptr& v, double beatIntervalSec){
  return make_shared<Act_RosPublish>(this, *v, beatIntervalSec);
}

Act::Ptr Roopi::rosPublish(const char* var_name, double beatIntervalSec){
  VariableBase::Ptr v = getVariable(var_name);
  return make_shared<Act_RosPublish>(this, *v, beatIntervalSec);
}

void Roopi::rosPublishAllVariables(){
  VariableBaseL vars = getVariables();
  for(VariableBase::Ptr* v:vars) permanentActs.append(rosPublish(*v));
}

void Roopi::reportCycleTimes(){
  threadReportCycleTimes();
}

arr Roopi::get_q0(){
  CHECK(s->q0.N, "kinematics needs to be set first");
  return s->q0;
}

WToken<mlr::KinematicWorld> Roopi::setK(){
  return s->modelWorld.set();
}

RToken<mlr::KinematicWorld> Roopi::getK(){
  return s->modelWorld.get();
}


Act_CtrlTask::Ptr Roopi::home(){
  return newCtrlTask(new TaskMap_qItself(), {2., .9, 1.}, get_q0());
}

Act_CtrlTask::Ptr Roopi::lookAt(const char* shapeName, double prec, const char* endeff_name){
  if(!endeff_name) endeff_name="endeffKinect";
  int cam = getK()->getFrameByName(endeff_name)->ID;
  int obj = getK()->getFrameByName(shapeName)->ID;
  return newCtrlTask(new TaskMap_Default(gazeAtTMT, cam, NoVector, obj), {}, {}, {prec});
}

Act_CtrlTask::Ptr Roopi::focusWorkspaceAt(const char* shapeName, double prec, const char* endeff_name){
  if(!endeff_name) endeff_name="endeffWorkspace";
  int ws  = getK()->getFrameByName(endeff_name)->ID;
  int obj = getK()->getFrameByName(shapeName)->ID;
  return newCtrlTask(new TaskMap_Default(posDiffTMT, ws, NoVector, obj), {}, {}, {2e-1});
}

Act_CtrlTask::Ptr Roopi::moveVel(const char* endeff_name, arr velocity){
  if(!endeff_name) endeff_name="endeffWorkspace";
  int eff  = getK()->getFrameByName(endeff_name)->ID;
  //lift hand
  auto lift = newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
  lift->set()->PD().setTarget(lift->task->y);
  lift->set()->PD().setGains(0, 10.);
  lift->set()->PD().v_target = velocity;
  return lift;
//  auto look = lookAt(objName);
//  mlr::wait(1.);
}


Act_CtrlTask::Ptr Roopi::newHoldingTask(){
  auto hold = make_shared<Act_CtrlTask>(this);
  hold->setMap(new TaskMap_qItself);
  hold->task->PD().setTarget( hold->y0 );
  hold->task->PD().setGains(30., 10.);
  hold->start();
  return hold;
}

Act_CtrlTask::Ptr Roopi::newCollisionAvoidance(){
  return newCtrlTask(new TaskMap_Proxy(allPTMT, {}, .05), {.1, .9}, {}, {1e2});
}

Act_CtrlTask::Ptr Roopi::newLimitAvoidance(){
  return newCtrlTask(new TaskMap_qLimits(getK()->getLimits()), {.1, .9}, {}, {1e2});
}

//Act_CtrlTask* Roopi::lookAt(const char* shapeName){
//  if(!shapeName){
//    if(s->_watchTask) s->_watchTask->stop();
//  }else{
//    if(!s->_watchTask){
//      int cam = getKinematics()->getShapeByName("endeffKinect")->index;
//      int obj = getKinematics()->getShapeByName(shapeName)->index;
//      s->_watchTask = new Act_CtrlTask(this, new TaskMap_Default(gazeAtTMT, cam, NoVector, obj));
//    }else{
//      auto task = s->_watchTask->set();
//      TaskMap_Default *map = dynamic_cast<TaskMap_Default*>(task->map);
//      CHECK(map,"");
//      map->j = getKinematics()->getShapeByName(shapeName)->index;
//      task->PD().setTarget( zeros(2) );
//    }
//    s->_watchTask->start();
//  }
//  return s->_watchTask;
//}

void Roopi::hold(bool still){
  if(!s->_holdPositionTask) s->_holdPositionTask = newHoldingTask();

  if(still){
    s->_holdPositionTask->set()->PD().setTarget(s->_holdPositionTask->task->y);
    s->_holdPositionTask->set()->PD().setGains(30., 10.);
    s->_holdPositionTask->start();
  }else{
    s->_holdPositionTask->stop();
  }
}

Act_CtrlTask::Ptr Roopi::collisions(bool on){
  if(!s->_collTask) s->_collTask = newCollisionAvoidance();
  if(on) s->_collTask->start();
  else s->_collTask->stop();
  return s->_collTask;
}

void Roopi::deactivateCollisions(const char* s1, const char* s2){
  s->modelWorld.writeAccess();
  mlr::Frame *sh1 = s->modelWorld().getFrameByName(s1);
  mlr::Frame *sh2 = s->modelWorld().getFrameByName(s2);
  if(sh1 && sh2) s->modelWorld().swift().deactivate(sh1, sh2);
  s->modelWorld.deAccess();
}

//==============================================================================
//
// basic CtrlTask management

Act_CtrlTask::Ptr Roopi::newCtrlTask(TaskMap* map, const arr& PD, const arr& target, const arr& prec){
  return make_shared<Act_CtrlTask>(this, map, PD, target, prec);
}

Act_CtrlTask::Ptr Roopi::newCtrlTask(const char* specs){
  return make_shared<Act_CtrlTask>(this, GRAPH(specs));
}



bool Roopi::useRos(){
  return s->useRos;
}

const mlr::String& Roopi::getRobot(){
  return s->robot;
}

Act_Thread::Ptr Roopi::newRosSpinner() {
  return make_shared<Act_Thread>(this, new RosCom_Spinner());
}

Act_Thread::Ptr Roopi::CameraView(const char* modelWorld_name){
  return make_shared<Act_Thread>(this, new ComputeCameraView(.2, modelWorld_name));
//  return Act_Thread::Ptr(new Act_Thread(this, {new ComputeCameraView(.2, modelWorld_name), new ImageViewer("kinect_rgb")}));
}

Act_PclPipeline Roopi::PclPipeline(bool view){
//  return Act_Thread(this, ::newPclPipeline(view));
  return Act_PclPipeline(this, view);
}

#if 0
Act_Group Roopi::PclPipeline(bool view){
  Act_Group G(this);
  G.append(new Kinect2PointCloud());
           threads.append(new Conv_arr_pcl("pclRawInput", "kinect_points", "kinect_rgb"));
           threads.append(new PclPipeline("pclRawInput"));
           if(view) threads.append(new PointCloudViewer());
         //  if(view) threads.append(new PclViewer("pclRawInput"));

}
#endif

Act_PerceptionFilter Roopi::PerceptionFilter(bool view){
//  return Act_Thread(this, ::newPerceptionFilter(view));
  return Act_PerceptionFilter(this, view);
}

Act_Thread::Ptr Roopi::PhysX(){
  return make_shared<Act_Thread>(this, newPhysXThread());
//  return Act_Th2<PhysXThread>(this, newPhysXThread());
}

Act_Thread::Ptr Roopi::GamepadControl(){
  return make_shared<Act_Thread>(this, new GamepadControlThread());
}


mlr::Frame *Roopi::newMarker(const char* name, const arr& pos){
  mlr::Frame *a;
  {
      auto K = setK();
      a = new mlr::Frame(K);
      a->name = name;
      a->X.pos = pos;
      mlr::Shape *sh = new mlr::Shape(*a);
      sh->type() = mlr::ST_marker;
      sh->mesh().C = {.8,0,0};
      sh->size()(0)=.1;
  }
  return a;
}

void Roopi::kinematicSwitch(const char* object, const char* attachTo, bool placing){
  {
    auto K = setK();
//    {
//      mlr::KinematicSwitch sw1(mlr::KinematicSwitch::deleteJoint, mlr::JT_none, NULL, object, K, 0);
//      sw1.apply(K);
//    }
    if(!placing){
      mlr::KinematicSwitch sw2(mlr::KinematicSwitch::addJointAtTo, mlr::JT_rigid, attachTo, object, K, 0);
      sw2.apply(K);
    }else{
      mlr::KinematicSwitch sw2(mlr::KinematicSwitch::addJointAtTo, mlr::JT_transXYPhi, attachTo, object, K, 0, NoTransformation, NoTransformation);
      sw2.apply(K);
    }
    K().checkConsistency();
    K().getJointState(); //enforces that the q & qdot are recalculated!
//    s->_ctrlView->get<OrsPoseViewer>()->recopyKinematics(K);
  }
}


