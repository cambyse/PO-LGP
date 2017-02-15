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
  if(_tweets) delete _tweets; _tweets=NULL;
  if(_tcm) delete _tcm; _tcm=NULL;
  if(_updater) delete _updater; _updater=NULL;

  if(ctrlView) delete ctrlView; ctrlView=NULL;
  threadCloseModules();
  cout << "bye bye" << endl;
}

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

    s->_tweets = new Act_Tweets(this);
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

  if(mlr::getParameter<bool>("useRos", false)){
    s->ctrlView = new Act_Thread(this, new OrsPoseViewer({"ctrl_q_ref", "ctrl_q_real"}, s->modelWorld.set(), .1), false);
  } else {
    s->ctrlView = new Act_Thread(this, new OrsPoseViewer({"ctrl_q_ref"}, s->modelWorld.set(), .1), false);
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

Act_TaskController& Roopi::taskController(){
  return *s->_tcm;
}

void Roopi::hold(bool still){
  if(still){
    s->ctrlTasks.writeAccess();
    for(CtrlTask *t:s->ctrlTasks()) t->active=false;
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
  sh->color[0]=.8; sh->color[1]=sh->color[2]=.0;
  sh->size[0]=.1;
  sh->X.pos = sh->rel.pos = pos;
  s->ctrlView->get<OrsPoseViewer>()->recopyKinematics(s->modelWorld());
  s->modelWorld.deAccess();
  return sh;
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


void Roopi::graspBox(const char* objName, bool rightNotLeft){

  //query some info from the kinematics first
  double width, above;
  uint obj, eff, grasp1, grasp2, cam, workspace;
  {
    auto K = getKinematics();

    //get obj size
    arr objSize(K().getShapeByName(objName)->size, 4, false);
    width = 2.*objSize(1);
    above = objSize(2);

    //relevant shapes
    obj = K().getShapeByName(objName)->index;
    if(s->robot=="pr2"){
      if(rightNotLeft){
        eff = K().getShapeByName("pr2R")->index;
        grasp1 = K().getJointByName("r_gripper_joint")->to->index;
        grasp2 = K().getJointByName("r_gripper_l_finger_joint")->to->index;
      }else{
        eff = K().getShapeByName("pr2L")->index;
        grasp1 = K().getJointByName("l_gripper_joint")->to->index;
        grasp2 = K().getJointByName("l_gripper_l_finger_joint")->to->index;
      }
      cam = K().getShapeByName("endeffKinect")->index;
      workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention, gripper positioning, alignment, open gripper
    auto look = newCtrlTask(new TaskMap_Default(gazeAtTMT, cam, NoVector, obj));
    auto ws = newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {1e1});
    auto up = newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos = newCtrlTask(new TaskMap_Default(posDiffTMT, eff, NoVector, obj), {}, {0.,0.,above+.1});
#if 1
    auto al1 = newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_x, obj, Vector_y) );
    auto al2 = newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_y, obj, Vector_x) );
#else
    auto al1 = newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_x, obj, Vector_x) );
    auto al2 = newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_y, obj, Vector_y) );
#endif
    double gripSize = width + .05;
    auto gripperR = newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    hold(false);
    wait({&pos, &gripperR, &look, &ws, &up});

    //lowering
    pos.set()->PD().setTarget( ARR(0,0,above-.03) );
    wait({&pos});

    //close gripper
    gripSize = width;
    gripperR.set()->PD().setTarget( {gripSize} );
    gripper2R.set()->PD().setTarget( {::asin(gripSize/(2.*.10))} );
    wait({&pos,&gripperR});

    //switch
    const char* effName = getKinematics()->shapes(eff)->name;
    kinematicSwitch(objName, effName);
    hold(true);
  }

  {
    //lift hand
    auto lift = newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift.set()->PD().setTarget(lift.task->y);
    lift.set()->PD().setGains(0, 10.);
    lift.set()->PD().v_target = ARR(0,0,.2);
    hold(false);
    mlr::wait(1.);
    hold(true);
  }
}

void Roopi::place(const char* objName, const char* ontoName){

  //query some info from the kinematics first
  double width, above;
  uint obj, onto, eff, grasp1, grasp2, cam, workspace;
  {
    auto K = getKinematics();

    //get obj size
    arr objSize(K().getShapeByName(objName)->size, 4, false);
    width = 2.*objSize(1);
    above = objSize(2);

    //relevant shapes
    mlr::Shape *ob = K().getShapeByName(objName);
    obj = ob->index;
    onto = K().getShapeByName(ontoName)->index;
    if(s->robot=="pr2"){
      mlr::Shape *sh = K().getShapeByName("pr2R");
      if(sh->body->index == ob->body->inLinks.scalar()->from->index){ //this is the right hand..
        eff = sh->index;
        grasp1 = K().getJointByName("r_gripper_joint")->to->index;
        grasp2 = K().getJointByName("r_gripper_l_finger_joint")->to->index;
      }else{
        sh = K().getShapeByName("pr2L");
        if(sh->body->index == ob->body->inLinks.scalar()->from->index){ //this is the left hand..
          eff = sh->index;
          grasp1 = K().getJointByName("l_gripper_joint")->to->index;
          grasp2 = K().getJointByName("l_gripper_l_finger_joint")->to->index;
        }else{
          HALT("which hand is this? Something's wrong");
        }
      }
      cam = K().getShapeByName("endeffKinect")->index;
      workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention & gripper positioning
    auto look = newCtrlTask(new TaskMap_Default(gazeAtTMT, cam, NoVector, obj));
    auto ws = newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {1e1});
    auto up = newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos = newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {0.,0.,above+.1});
    hold(false);
    wait({&look, &ws, &up, &pos});

    //lowering
    pos.set()->PD().setTarget( ARR(0,0,above) );
    wait({&pos});

    //switch
    kinematicSwitch(objName, ontoName);

    //open gripper
    double gripSize = width + .05;
    auto gripperR = newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    wait({&gripperR});
  }

  {
    //lift hand
    auto lift = newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift.set()->PD().setTarget(lift.task->y);
    lift.set()->PD().setGains(0, 10.);
    lift.set()->PD().v_target = ARR(0,0,.2);
    hold(false);
    mlr::wait(1.);
    hold(true);
  }
}
