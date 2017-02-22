#include "roopi.h"
#include "roopi-private.h"

#include <Perception/viewer.h>

#define baxter 0

//==============================================================================

Roopi_private::Roopi_private(Roopi* roopi)
  : modelWorld(NULL, "modelWorld"){
}

Roopi_private::~Roopi_private(){
  modulesReportCycleTimes();

  //delete persistant acts
  if(_ComRos) delete _ComRos; _ComRos=NULL; //shut of the spinner BEFORE you close the pubs/subscribers..
  if(_ComPR2) delete _ComPR2; _ComPR2=NULL;
  if(_holdPositionTask) delete _holdPositionTask; _holdPositionTask=NULL;
  if(_watchTask) delete _watchTask; _watchTask=NULL;
  if(_collTask) delete _collTask; _collTask=NULL;
  if(_tweets) delete _tweets; _tweets=NULL;
  if(_tcm) delete _tcm; _tcm=NULL;
  if(_updater) delete _updater; _updater=NULL;

  if(ctrlView) delete ctrlView; ctrlView=NULL;
  threadCloseModules();
  cout << "bye bye" << endl;
}

//==============================================================================

template<class T> struct PersistScope : T{
  PersistScope(T act):T(act){}
  ~PersistScope(){}
};

//==============================================================================

Roopi::Roopi(bool autoStartup)
  : s(new Roopi_private(this)), acts(NULL, "acts"){
  s->model = mlr::getParameter<mlr::String>("model", "model.g");
  s->robot = mlr::getParameter<mlr::String>("robot", "pr2");
  s->useRos = mlr::getParameter<bool>("useRos", false);

  if(autoStartup){
    if(s->useRos){
      s->_ComRos = new Act_ComRos(this);
      s->_ComPR2 = new Act_ComPR2(this);
    }

    startTweets();
    setKinematics(s->model);
    startTaskController();
  }
}

Roopi::~Roopi(){
  delete s;
}

void Roopi::setKinematics(const char* filename){
  mlr::String name(filename);
  mlr::KinematicWorld K;
  if(name=="pr2") {
    K.init(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  } else if(name=="baxter") {
    K.init(mlr::mlrPath("data/baxter_model/baxter.ors").p);
  } else {
    K.init(name);
  }

  setKinematics(K);
}

void Roopi::setKinematics(const mlr::KinematicWorld& K){
//  CHECK(s->modelWorld.get()->q.N==0, "has been set before???");
  s->modelWorld.set() = K;

  if(s->useRos){
    s->ctrlView = new Act_Thread(this, new OrsPoseViewer("modelWorld", {"ctrl_q_ref", "ctrl_q_real"}, .1), false);
  } else {
    s->ctrlView = new Act_Thread(this, new OrsPoseViewer("modelWorld", {"ctrl_q_ref"}, .1), false);
//    s->ctrlView = new Act_Thread(this, new OrsViewer("modelWorld", .1), false);
  }
}

Act_TaskController& Roopi::startTaskController(){
  if(!s->_holdPositionTask) s->_holdPositionTask = new Act_CtrlTask(this);
  s->_holdPositionTask->setMap(new TaskMap_qItself);
  s->_holdPositionTask->set()->PD().setTarget( s->_holdPositionTask->y0 );
  s->_holdPositionTask->set()->PD().setGains(30., 10.);
  s->_holdPositionTask->start();

  s->_updater = new Act_Thread(this, new CtrlTaskUpdater, true);

  s->_tcm = new Act_TaskController(this);
  return *s->_tcm;
}

Act* Roopi::startTweets(bool go){
  if(!s->_tweets && go) s->_tweets = new Act_Tweets(this);
  if(s->_tweets && !go){ delete s->_tweets; s->_tweets=NULL; }
  return s->_tweets;
}

Act_TaskController& Roopi::taskController(){
  return *s->_tcm;
}

void Roopi::hold(bool still){
  if(still){
    s->ctrlTasks.writeAccess();
    for(CtrlTask *t:s->ctrlTasks()){
      if(!s->_collTask || t!=s->_collTask->task)
        t->active=false;
    }
    s->ctrlTasks.deAccess();

    s->_holdPositionTask->set()->PD().setTarget(s->_holdPositionTask->task->y);
    s->_holdPositionTask->set()->PD().setGains(30., 10.);
    s->_holdPositionTask->start();

  }else{
    s->_holdPositionTask->stop();
  }
}

Act_CtrlTask* Roopi::home(){
  s->ctrlTasks.writeAccess();
  for(CtrlTask *t:s->ctrlTasks()) t->active=false;
  s->ctrlTasks.deAccess();

  s->_holdPositionTask->set()->PD().setTarget( s->_holdPositionTask->y0 );
  s->_holdPositionTask->set()->PD().setGainsAsNatural(2.,.9);
  s->_holdPositionTask->set()->PD().maxVel=1.;
  s->_holdPositionTask->start();

  return s->_holdPositionTask;
}

Act_CtrlTask* Roopi::lookAt(const char* shapeName){
  if(!shapeName){
    if(s->_watchTask) s->_watchTask->stop();
  }else{
    if(!s->_watchTask){
      int cam = getKinematics()->getShapeByName("endeffKinect")->index;
      int obj = getKinematics()->getShapeByName(shapeName)->index;
      s->_watchTask = new Act_CtrlTask(this, new TaskMap_Default(gazeAtTMT, cam, NoVector, obj));
    }else{
      auto task = s->_watchTask->set();
      TaskMap_Default *map = dynamic_cast<TaskMap_Default*>(task->map);
      CHECK(map,"");
      map->j = getKinematics()->getShapeByName(shapeName)->index;
      task->PD().setTarget( zeros(2) );
    }
    s->_watchTask->start();
  }
  return s->_watchTask;
}

Act_CtrlTask* Roopi::collisions(bool on){
  if(!s->_collTask){
    s->_collTask = new Act_CtrlTask(std::move(newCollisionAvoidance()));
//    s->_collTask->set()->hierarchy=2;
//    s->_collTask = new Act_CtrlTask(this, new TaskMap_Proxy(allPTMT, {}, .05), {.1, .9});
  }

  if(on) s->_collTask->start();
  else s->_collTask->stop();
  return s->_collTask;
}

Act_CtrlTask Roopi::newCollisionAvoidance(){
  return Act_CtrlTask(this, new TaskMap_Proxy(allPTMT, {}, .05), {.1, .9}, {}, {1e2});
}

Act_CtrlTask Roopi::newLimitAvoidance(){
  return Act_CtrlTask(this, new TaskMap_qLimits(getKinematics()->getLimits()), {.1, .9}, {}, {1e2});
}

WToken<mlr::KinematicWorld> Roopi::setKinematics(){
  return s->modelWorld.set();
}

RToken<mlr::KinematicWorld> Roopi::getKinematics(){
  return s->modelWorld.get();
}

//RToken<mlr::Shape> Roopi::getShape(const char* name){
//  s->modelWorld.readAccess();
//  mlr::Shape *sh = s->modelWorld().getShapeByName(name);
//  CHECK(sh, "Shape not found");
//  return RToken<mlr::Shape>(*s->modelWorld.data, sh, NULL, NULL, true);
//}

//RToken<mlr::Joint> Roopi::getJoint(const char* name){
//  s->modelWorld.readAccess();
//  mlr::Joint *j = s->modelWorld().getJointByName(name);
//  CHECK(j, "Joint not found");
//  return RToken<mlr::Joint>(*s->modelWorld.data, j, NULL, NULL, true);
//}

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

Act_Thread Roopi::newCameraView(){
  ImageViewer *v = new ImageViewer("cameraView");
  v->flipImage = true;
  v->threadOpen(true);
  return Act_Thread(this, new ComputeCameraView(30), false);
}

mlr::Shape* Roopi::newMarker(const char* name, const arr& pos){
  s->modelWorld.writeAccess();
  mlr::Shape *sh = new mlr::Shape(s->modelWorld(), NoBody);
  sh->name = name;
  sh->type = mlr::ST_marker;
  sh->color[0]=.8; sh->color[1]=sh->color[2]=.0; sh->color[3]=1.;
  sh->size[0]=.1;
  sh->X.pos = sh->rel.pos = pos;
  s->modelWorld.deAccess();
  s->ctrlView->get<OrsPoseViewer>()->recopyKinematics(); //s->modelWorld());
  return sh;
}

void Roopi::resyncView(){
  s->ctrlView->get<OrsPoseViewer>()->recopyKinematics(); //s->modelWorld());
}

void Roopi::kinematicSwitch(const char* object, const char* attachTo){
  s->modelWorld.writeAccess();
  mlr::KinematicSwitch sw1(mlr::KinematicSwitch::deleteJoint, mlr::JT_none, NULL, object, s->modelWorld(), 0);
  mlr::KinematicSwitch sw2(mlr::KinematicSwitch::addJointAtTo, mlr::JT_rigid, attachTo, object, s->modelWorld(), 0);
  sw1.apply(s->modelWorld());
  sw2.apply(s->modelWorld());
  s->modelWorld().getJointState(); //enforces that the q & qdot are recalculated!
  s->ctrlView->get<OrsPoseViewer>()->recopyKinematics(s->modelWorld());
  s->modelWorld.deAccess();
}


