#include "roopi.h"
#include "roopi-private.h"

#include <Gui/viewer.h>
#include <Kin/PhysXThread.h>
#include <Control/GamepadControlThread.h>
#include <RosCom/spinner.h>
#include <Perception/roopi_Perception.h>
#include <PCL/roopi_PCL.h>

#define baxter 0

//==============================================================================

Roopi_private::Roopi_private(Roopi* roopi)
  : modelWorld(NULL, "modelWorld"){
}

Roopi_private::~Roopi_private(){
  threadReportCycleTimes();

  //delete persistant acts
  if(_tweets) delete _tweets; _tweets=NULL; //somehow the tweeter crashes to tweet the other's kill...
  if(_ComRos) delete _ComRos; _ComRos=NULL; //shut of the spinner BEFORE you close the pubs/subscribers..
  if(_ComPR2) delete _ComPR2; _ComPR2=NULL;
  if(_holdPositionTask) delete _holdPositionTask; _holdPositionTask=NULL;
  if(_watchTask) delete _watchTask; _watchTask=NULL;
  if(_collTask) delete _collTask; _collTask=NULL;
  if(_taskController) delete _taskController; _taskController=NULL;
  if(_taskUpdater) delete _taskUpdater; _taskUpdater=NULL;

  if(_ctrlView) delete _ctrlView; _ctrlView=NULL;
  threadCloseModules();
  cout << "bye bye" << endl;
}

//==============================================================================

Roopi::Roopi(bool autoStartup, bool controlView)
  : s(new Roopi_private(this)), acts(NULL, "acts"){
  s->model = mlr::getParameter<mlr::String>("model", "model.g");
  s->robot = mlr::getParameter<mlr::String>("robot", "pr2");
  s->useRos = mlr::getParameter<bool>("useRos", false);

  if(autoStartup){
    if(s->useRos){
      s->_ComRos = new Act_Thread(this, new RosCom_Spinner());
      s->_ComPR2 = new Act_ComPR2(this);
    }

    startTweets();
    setKinematics(s->model, controlView);
    startTaskController();
  }
}

Roopi::~Roopi(){
  delete s;
}

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
    if(s->useRos){
      s->_ctrlView = new Act_Thread(this, new OrsPoseViewer("modelWorld", {"ctrl_q_ref", "ctrl_q_real"}, .1));
    } else {
      s->_ctrlView = new Act_Thread(this, new OrsPoseViewer("modelWorld", {"ctrl_q_ref"}, .1));
    }
  }
}

Act_TaskController& Roopi::startTaskController(){
//  s->_taskUpdater = new Act_Thread(this, new CtrlTaskUpdater);
  s->_taskController = new Act_TaskController(this);
  return *s->_taskController;
}

Act_Tweets& Roopi::startTweets(bool go){
  if(!s->_tweets && go) s->_tweets = new Act_Tweets(this);
  if(s->_tweets && !go){ delete s->_tweets; s->_tweets=NULL; }
  return *s->_tweets;
}

Act_TaskController& Roopi::getTaskController(){
  return *s->_taskController;
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


Act_CtrlTask Roopi::home(){
  return Act_CtrlTask(this, new TaskMap_qItself(), {2., .9, 1.}, get_q0());
}

Act_CtrlTask Roopi::lookAt(const char* shapeName, double prec){
  int cam = getK()->getShapeByName("endeffKinect")->index;
  int obj = getK()->getShapeByName(shapeName)->index;
  return Act_CtrlTask(this, new TaskMap_Default(gazeAtTMT, cam, NoVector, obj), {}, {}, {prec});
}

Act_CtrlTask Roopi::newHoldingTask(){
  auto hold = Act_CtrlTask(this);
  hold.setMap(new TaskMap_qItself);
  hold.task->PD().setTarget( hold.y0 );
  hold.task->PD().setGains(30., 10.);
  hold.start();
  return hold;
}

Act_CtrlTask Roopi::newCollisionAvoidance(){
  return Act_CtrlTask(this, new TaskMap_Proxy(allPTMT, {}, .05), {.1, .9}, {}, {1e2});
}

Act_CtrlTask Roopi::newLimitAvoidance(){
  return Act_CtrlTask(this, new TaskMap_qLimits(getK()->getLimits()), {.1, .9}, {}, {1e2});
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
  if(!s->_holdPositionTask) s->_holdPositionTask = new Act_CtrlTask(std::move(newHoldingTask()));

  if(still){
    s->_holdPositionTask->set()->PD().setTarget(s->_holdPositionTask->task->y);
    s->_holdPositionTask->set()->PD().setGains(30., 10.);
    s->_holdPositionTask->start();
  }else{
    s->_holdPositionTask->stop();
  }
}

Act_CtrlTask* Roopi::collisions(bool on){
  if(!s->_collTask) s->_collTask = new Act_CtrlTask(std::move(newCollisionAvoidance()));
  if(on) s->_collTask->start();
  else s->_collTask->stop();
  return s->_collTask;
}

//==============================================================================
//
// basic CtrlTask management

Act_CtrlTask Roopi::newCtrlTask(TaskMap* map, const arr& PD, const arr& target, const arr& prec){
  return Act_CtrlTask(this, map, PD, target, prec);
}

Act_CtrlTask Roopi::newCtrlTask(const char* specs){
  return Act_CtrlTask(this, GRAPH(specs));
}

bool Roopi::wait(std::initializer_list<Act*> acts, double timeout){
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
#else
  double startTime = mlr::realTime();
  ConditionVariable waiter;
  for(Act *act : acts) waiter.listenTo(act);
  for(;;){
    bool allConv = true;
    for(Act *act : acts) if(act->getStatus()<=0) allConv=false;
    if(allConv) return true;

    waiter.statusLock();
    // The typical pattern (also in Thread): CondVar status==0 -> idle; CondVar status>0 -> there is work to do (and 'messengers' lists who set it to 1)
    if(waiter.status==0){ //no new signals
      if(timeout>0.){
        if(mlr::realTime()-startTime > timeout){ waiter.mutex.unlock(); return false; }
        if(!waiter.waitForSignal(timeout, true)){
          cout << "not converged, timeout reached" << endl;
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
#endif
}

Act_Script Roopi::runScript(const std::function<int ()>& script){
  return Act_Script(this, script);
}

const mlr::String& Roopi::getRobot(){
  return s->robot;
}

Act_Thread Roopi::newComROS(){
  return Act_Thread(this, new RosCom_Spinner());
}

Act_Thread Roopi::newCameraView(bool view){
  if(!view) return Act_Thread(this, {new ComputeCameraView(.2)});
  return Act_Thread(this, {new ComputeCameraView(.2), new ImageViewer("kinect_rgb")});
}

Act_Thread Roopi::newPclPipeline(bool view){
  return Act_Thread(this, ::newPclPipeline(view));
}

Act_Thread Roopi::newPerceptionFilter(bool view){
  return Act_Thread(this, ::newPerceptionFilter(view));
}

Act_Thread Roopi::newPhysX(){
  return Act_Thread(this, newPhysXThread());
}

Act_Thread Roopi::newGamepadControl(){
  return Act_Thread(this, new GamepadControlThread());
}


mlr::Shape* Roopi::newMarker(const char* name, const arr& pos){
  mlr::Shape *sh;
  {
    auto K = setK();
    sh = new mlr::Shape(K, NoBody);
    sh->name = name;
    sh->type = mlr::ST_marker;
    sh->color[0]=.8; sh->color[1]=sh->color[2]=.0; sh->color[3]=1.;
    sh->size[0]=.1;
    sh->X.pos = sh->rel.pos = pos;
  }
  resyncView();
  return sh;
}

void Roopi::resyncView(){
  s->_ctrlView->get<OrsPoseViewer>()->recopyKinematics();
}

void Roopi::kinematicSwitch(const char* object, const char* attachTo){
  {
    auto K = setK();
    mlr::KinematicSwitch sw1(mlr::KinematicSwitch::deleteJoint, mlr::JT_none, NULL, object, K, 0);
    mlr::KinematicSwitch sw2(mlr::KinematicSwitch::addJointAtTo, mlr::JT_rigid, attachTo, object, K, 0);
    sw1.apply(K);
    sw2.apply(K);
    K().getJointState(); //enforces that the q & qdot are recalculated!
//    s->_ctrlView->get<OrsPoseViewer>()->recopyKinematics(K);
  }
  resyncView();
}


