#include "roopi.h"
#include "roopi-private.h"

#include <Algo/spline.h>
#include <Control/TaskControlThread.h>
#include <Motion/motion.h>
#include <Optim/lagrangian.h>
#include <Gui/viewer.h>


#define baxter 0

//==============================================================================
/*

(mt) The mid-term concept should be: Roopi allows you to create (and return) little
handles on 'tasks' or 'things'. These tasks or things can be:
-- control tasks
-- a task following
-- trajectories
-- motion planning problems
-- system level processes (like subscriptions, perceptual processes, etc)
-- interfaces (gamepad, orsviewers, etc)

The template for now is

auto* handle = Roopi::createX(...)
handle->setParameters
handle->run
handle->quit or Roopi::destroyX(handle)

Maybe plan:
-- first just control tasks & motion planning
-- make path following a thread
-- make logging just such a thread and 'handle'
-- next the system processes (perception threads)
-- the activities we already have

*/
//==============================================================================

Roopi_private::Roopi_private(Roopi* roopi)
  : modelWorld(NULL, "modelWorld"),
    _holdPositionTask(roopi)
  #if baxter
  spctb(tcm.realWorld),
    #endif
{
#if baxter
  if(mlr::getParameter<bool>("useRos", false)){
    nh = new ros::NodeHandle;
    pub = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
  }
#endif
}

Roopi_private::~Roopi_private(){
  modulesReportCycleTimes();

  //delete persistant acts
  if(_ComRos) delete _ComRos; //shut of the spinner BEFORE you close the pubs/subscribers..
  if(_ComPR2) delete _ComPR2;
  _holdPositionTask.kill();
  if(_tcm) delete _tcm;

  if(ctrlView){ ctrlView->threadClose(); delete ctrlView; }
  //    if(holdPositionTask) delete holdPositionTask;
  //    if(holdPositionTask2) delete holdPositionTask2;
  threadCloseModules();
  cout << "bye bye" << endl;
}

//==============================================================================

Roopi::Roopi(bool autoStartup)
  : s(new Roopi_private(this)) {
  if(autoStartup){
    mlr::String model = mlr::getParameter<mlr::String>("model", "model.g");
    mlr::String robot = mlr::getParameter<mlr::String>("robot", "pr2");
    bool useRos = mlr::getParameter<bool>("useRos", false);

    if(useRos){
      s->_ComRos = new Act_ComRos(this);
      s->_ComPR2 = new Act_ComPR2(this);
    }

    setKinematics(model);
    startTaskController();
  }
}


Roopi::Roopi(mlr::KinematicWorld& world)
  : s(new Roopi_private(this)){
  setKinematics(world);
  startTaskController();
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
  CHECK(s->modelWorld.get()->q.N==0, "has been set before???");
  s->modelWorld.set() = K;

  if(mlr::getParameter<bool>("oldfashinedTaskControl")) {
    s->ctrlView = new OrsPoseViewer({"ctrl_q_ref", "ctrl_q_real"}, s->modelWorld.set());
  } else {
    s->ctrlView = new OrsPoseViewer({"ctrl_q_real"}, s->modelWorld.set());
  }
}

Act_TaskController& Roopi::startTaskController(){
  s->_holdPositionTask.setMap(new TaskMap_qItself);
  s->_holdPositionTask.set()->y_ref = s->_holdPositionTask.y0;
  s->_holdPositionTask.set()->setGains(30., 10.);
  s->_holdPositionTask.start();

//  CHECK(!s->tcm,"");
//  s->tcm = new TaskControlThread("none", NoWorld);
//  s->tcm->threadLoop();
//  s->tcm->waitForOpened();
//  return {s->tcm->name, s->tcm};

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

    s->_holdPositionTask.set()->setTargetToCurrent();
    s->_holdPositionTask.start();

  }else{
    s->_holdPositionTask.stop();
  }
}

Act_CtrlTask *Roopi::home(){
  s->ctrlTasks.writeAccess();
  for(CtrlTask *t:s->ctrlTasks()) t->active=false;
  s->ctrlTasks.deAccess();

  s->_holdPositionTask.set()->y_ref = s->_holdPositionTask.y0;
  s->_holdPositionTask.start();

  return &s->_holdPositionTask;
}

WToken<mlr::KinematicWorld> Roopi::setKinematics(){
  return s->modelWorld.set();
}

RToken<mlr::KinematicWorld> Roopi::getKinematics(){
  return s->modelWorld.get();
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
#if 1
  double startTime = mlr::realTime();
  for(;;){
    bool allConv = true;
    for(Act *act : acts){
      if(act->getStatus()<=0){
        allConv = false;
        break;
      }
    }
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
  for(Act *act : acts) act->status.listeners.append(waiter);
  for(bool go=true;go;){
    if(timeout>0){
      if(mlr::realTime()-startTime > timeout) return false;
      if(!waiter.waitForSignal(timeout, false)) return false;
    }else{
      waiter.waitForSignal();
    }
    for(Act *act : acts) if(act->status.getStatus()!=0){ go=false; break; }
  }
  return true;
#endif
}

void Roopi::newCameraView(){
  ImageViewer *v = new ImageViewer("cameraView");
  v->flipImage = true;
  v->threadOpen(true);
  new ComputeCameraView(30);
}

mlr::Shape* Roopi::newMarker(const char* name, const arr& pos){
  s->modelWorld.writeAccess();
  mlr::Shape *sh = new mlr::Shape(s->modelWorld(), NoBody);
  sh->name = name;
  sh->type = mlr::ST_marker;
  sh->color[0]=.8; sh->color[1]=sh->color[2]=.0;
  sh->size[0]=.1;
  sh->X.pos = sh->rel.pos = pos;
  s->ctrlView->recopyKinematics(s->modelWorld());
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
  s->ctrlView->recopyKinematics(s->modelWorld());
  s->modelWorld.deAccess();
}




#if 0
CtrlTask* Roopi::createCtrlTask(const char* name, TaskMap* map, bool active) {
  CtrlTask* ct = new CtrlTask(name, map);
  map->phi(ct->y, NoArr, getKinematics()); // initialize with the current value. TODO taskControllerModule updates these only if they are active
  ct->y_ref = ct->y;
  ct->active = active;
  if(active) ct->setGainsAsNatural(1., .8);
  else       ct->setGains(0.0,0.0);
  ct->setC(eye(ct->y_ref.d0)*1000.0); //TODO
  s->ctrlTasks.set()->append(ct);
  return ct;
}

void Roopi::activateCtrlTask(CtrlTask* t, bool reinitializeReferences){
  s->ctrlTasks.writeAccess();
  if(reinitializeReferences) {
    arr currentValue = getTaskValue(t);
    t->y = currentValue;
    t->y_ref = currentValue;
  }
  t->active = true;
  s->ctrlTasks.deAccess();
}

void Roopi::deactivateCtrlTask(CtrlTask* t) {
  s->ctrlTasks.writeAccess();
  t->active = false;
  s->ctrlTasks.deAccess();
}

void Roopi::destroyCtrlTask(CtrlTask* t) {
  s->ctrlTasks.set()->removeValue(t);
  delete &t->map;
  delete t;
}

void Roopi::modifyCtrlTaskReference(CtrlTask* ct, const arr& yRef, const arr& yDotRef) {
  s->ctrlTasks.writeAccess();
  ct->setTarget(yRef, yDotRef);
  s->ctrlTasks.deAccess();
}

void Roopi::holdPosition() {
  s->ctrlTasks.writeAccess();
  for(CtrlTask *t:s->ctrlTasks()) t->active=false;
  s->ctrlTasks.deAccess();

  /*CtrlTask* ct = createCtrlTask("HoldPosition", new TaskMap_qItself);
  modifyCtrlTaskGains(ct, 30.0, 5.0);
  modifyCtrlC(ct, ARR(1000.0));
  activateCtrlTask(ct);*/

  //modifyCtrlTaskReference(holdPositionTask, getKinematics()->getJointState());
//  activateCtrlTask(s->holdPositionTask, true);
  s->holdPositionTask2.start();
}

void Roopi::releasePosition() {
//  deactivateCtrlTask(s->holdPositionTask);
  s->holdPositionTask2.stop();
}

bool Roopi::converged(CtrlTask* ct, double tolerance) {
  bool conv = false;
  s->ctrlTasks.readAccess();
  if(ct->isConverged(tolerance)) conv = true;
  s->ctrlTasks.deAccess();
  return conv;
}

bool Roopi::waitForConv(CtrlTask* ct, double maxTime, double tolerance) {
  return waitForConv(CtrlTaskL({ct}), maxTime, tolerance);
}

bool Roopi::waitForConv(const CtrlTaskL& cts, double maxTime, double tolerance) {
  double startTime = mlr::timerRead();
  while(true) {
    bool allConv = true;
    s->ctrlTasks.readAccess();
    for(CtrlTask* t : cts) {
      if(!t->isConverged(tolerance)) {
        allConv = false;
        break;
      } else {
        cout << t->name << " has converged" << endl;
      }
    }
    s->ctrlTasks.deAccess();
    if(allConv) return true;
    double actTime = mlr::timerRead() - startTime;
    if(maxTime > -1.0 && actTime > maxTime) {
      cout << "not converged, timeout reached" << endl;
      return false;
    }
    mlr::wait(0.1);
  }
  return false;
}

bool Roopi::waitForConvTo(CtrlTask* ct, const arr& desState, double maxTime, double tolerance) {
  double startTime = mlr::timerRead();
  while(true) {
    bool conv = false;
    s->ctrlTasks.readAccess();
    if((ct->y.N && maxDiff(ct->y, desState) < tolerance)) conv = true;
    cout << ct->error() << endl;
    s->ctrlTasks.deAccess();
    if(conv) return true;
    double actTime = mlr::timerRead() - startTime;
    if(maxTime > -1.0 && actTime > maxTime) {
      cout << "not converged, timeout reached" << endl;
      return false;
    }
    mlr::wait(0.1);
  }
  return false;
}



TaskControlThread* Roopi::tcm() {
  return s->tcm->tcm;
}

void Roopi::interpolateToReferenceThread(CtrlTask* task, double executionTime, const arr& reference, const arr& start) {

}

TaskReferenceInterpolAct*Roopi::createTaskReferenceInterpolAct(const char* name, CtrlTask* task) {
  return new TaskReferenceInterpolAct(*this, name, task);
}

void Roopi::interpolateToReference(TaskReferenceInterpolAct* t, double executionTime, const arr& reference, const arr& start) {
  t->startExecution(executionTime, reference, start);
}

bool Roopi::waitForFinishedTaskReferenceInterpolAct(TaskReferenceInterpolAct* t, double maxTime) {
  double startTime = mlr::timerRead();
  while(true) {
    mlr::wait(0.1);
    if(t->isIdle() || t->isClosed()) break;
    double actTime = mlr::timerRead() - startTime;
    if(maxTime > -1.0 && maxTime > actTime) {
      return false;
    }
  }
  return true;
}

void Roopi::interpolateToReference(CtrlTask* task, double executionTime, const arr& reference, const arr& start) {
  cout << "start interpolating to target" << endl;
  arr initialRef;
  if(&start) {
    initialRef = start;
  } else {
    initialRef = getTaskValue(task);
  }
  double startTime = mlr::timerRead();
  double time = 0.0;
  while(true) {
    time = mlr::timerRead() - startTime;
    double s = time/executionTime;
    if(s > 1.0) {
      cout << "finished interpolating to target" << endl;
      break;
    }
    arr actRef = initialRef + (reference - initialRef)*0.5*(1.0-cos(MLR_PI*s)); //TODO is this a good motion profile? Robotics lecture says yes :-)
    modifyCtrlTaskReference(task, actRef);
  }
  modifyCtrlTaskReference(task, reference);
}

//==============================================================================

arr Roopi::getJointState() {
  if(tcm()->oldfashioned && !tcm()->useRos) {
    return getKinematics()->getJointState();
  }
  return tcm()->ctrl_obs.get()->q;
}

arr Roopi::getJointSign() {
  return tcm()->qSign.get();
}

arr Roopi::getTorques() {
  return tcm()->ctrl_obs.get()->u_bias;
}

arr Roopi::getFTLeft() {
  return tcm()->ctrl_obs.get()->fL;
}

arr Roopi::getFTRight() {
  return tcm()->ctrl_obs.get()->fR;
}

arr Roopi::getTaskValue(CtrlTask* task) {
  arr y;
  task->map.phi(y, NoArr, getKinematics()); //TODO or better with realWorld?
  return y;
}

//==============================================================================

void Roopi::copyModelWorld(mlr::KinematicWorld& K) {
  K = s->modelWorld.get();
}

void Roopi::syncWorldWithReal(mlr::KinematicWorld& K) {
  arr qK;
  //this syncs with the real world, except for the case where no real world is available. TODO does this make sense?
  if(tcm()->oldfashioned && !tcm()->useRos) {
    transferQbetweenTwoWorlds(qK, getKinematics()->getJointState(), K, getKinematics());
  } else {
    transferQbetweenTwoWorlds(qK, tcm()->ctrl_obs.get()->q, K, getKinematics());
  }
  K.setJointState(qK);
}

#if 0 //planWorld!!

mlr::KinematicWorld& Roopi::getPlanWorld() {
  return planWorld;
}

double Roopi::getLimitConstraint(double margin) {
  LimitsConstraint limit(margin);
  arr y;
  syncPlanWorld();
  limit.phi(y, NoArr, planWorld);
  return y.first();
}

double Roopi::getCollisionConstraint(double margin) {
  CollisionConstraint collisions(margin);
  arr y;
  syncPlanWorld();
  collisions.phi(y, NoArr, planWorld);
  return y.first();
}
#endif

void Roopi::followTaskTrajectory(CtrlTask* task, double executionTime, const arr& trajectory) {
  followTaskTrajectories(CtrlTaskL({task}), executionTime, {trajectory});
}

void Roopi::followTaskTrajectories(const CtrlTaskL& tasks, double executionTime, const arrA& trajY, const arrA& trajYDot, const arrA& trajYDDot) {
  //TODO not reasonable yet
  mlr::Array<mlr::Spline> ySplines;
  mlr::Array<mlr::Spline> yDotSplines;
  mlr::Array<mlr::Spline> yDDotSplines;

  for(uint i = 0; i < tasks.N; i++) { //Not sure if the iterator returns elements in FIFO order, therefore "for" loop
    if(trajY.N) {
      ySplines.append(mlr::Spline(trajY(i).d0, trajY(i)));
    }
    if(trajYDot.N) {
      yDotSplines.append(mlr::Spline(trajYDot(i).d0, trajYDot(i)));
    }
    if(trajYDDot.N) {
      yDDotSplines.append(mlr::Spline(trajYDDot(i).d0, trajYDDot(i)));
    }
  }

#if 1 //marc's version
  s->ctrlTasks.writeAccess();
  for(CtrlTask *t:s->ctrlTasks()) t->active=false;
  for(CtrlTask *t:tasks) t->active=true; //they were 'added' to the ctrlTasks list on creation!!
//  tcm()->verbose=1;
  s->ctrlTasks.deAccess();
#else //danny's
  s->ctrlTasks.set()->clear(); // TODO memory leak! The question is if it is a good idea to delete all tasks here, because one could still use them in the main.cpp
  addCtrlTasks(tasks);
#endif

  double startTime = mlr::timerRead();
  double time = 0.0;
  uint n = 0;
  cout << "start execution of trajectory" << endl;
  while(true) { //(mt) oh no! This goes full throttle!!
    time = mlr::timerRead() - startTime;
//    cout << time << endl;
    double s = time/executionTime;
    if(s > 1.) {
      cout << "finished execution of trajectory" << endl;
      break;
    }
    for(uint i = 0; i < tasks.N; i++) {
      modifyCtrlTaskReference(tasks(i), ySplines(i).eval(s));
    }
    n++;
  }
  for(uint i = 0; i < tasks.N; i++) {
    modifyCtrlTaskReference(tasks(i), ySplines(i).eval(1.0));
  }
}

void Roopi::followQTrajectory(const Roopi_Path* path) {
  CtrlTask* ct = createCtrlTask("followQTrajectory", new TaskMap_qItself);
  ct->setC(ARR(1000.0));
  ct->setGains(ARR(30.0), ARR(5.0));
  followTaskTrajectory(ct, path->executionTime, path->path);
  destroyCtrlTask(ct); //TODO this is a bit unsmooth, because then no task is active anymore! Calling holdPosition directly afterwards works, bit is a bit unsmmooth
}

Roopi_Path* Roopi::createPathInJointSpace(CtrlTask* task, double executionTime, bool verbose) {
  return createPathInJointSpace(CtrlTaskL({task}), executionTime, verbose);
}

Roopi_Path* Roopi::createPathInJointSpace(const CtrlTaskL& tasks, double executionTime, bool verbose) {
  Roopi_Path *path = new Roopi_Path(*this, executionTime);

  syncWorldWithReal(path->K);
  MotionProblem MP(path->K);

  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(MP.world), OT_sumOfSqr);
  t->map.order=2; //acceleration task
  t->setCostSpecs(0, MP.T, {0.}, 1.0);

  t = MP.addTask("collisions", new CollisionConstraint(0.11), OT_ineq);
  t->setCostSpecs(0., MP.T, {0.}, 1.0);
  t = MP.addTask("qLimits", new LimitsConstraint(0.05), OT_ineq); //TODO!!!!!!!!!!!!!!! margin
  t->setCostSpecs(5, MP.T, {0.}, 1.0);

  for(CtrlTask* ct : tasks) {
    t = MP.addTask(ct->name, &ct->map, OT_sumOfSqr);
    t->setCostSpecs(MP.T-5, MP.T, ct->y_ref, 5.0); //TODO MP.T-how many? TODO ct->get_y_ref refactor!
  }

  path->path = MP.getInitialization();

  optConstrained(path->path , NoArr, Convert(MP.komo_problem), OPT(verbose=verbose)); //TODO options
  //if(verbose) MP.reportFull(); //TODO fails
  Graph result = MP.getReport();
  path->path.reshape(MP.T, path->path.N/MP.T);
  path->cost = result.get<double>({"total","sqrCosts"});
  path->constraints = result.get<double>({"total","constraints"});

  if(path->constraints < .1 && path->cost < 5.) {
    path->isGood=true;
  } else {
    path->isGood=false;
    cout << "No reasonable trajectory found!" << endl;
  }

  return path;
}

bool Roopi::goToPosition(const arr& pos, const char* shape, double executionTime, bool verbose) {
  CtrlTask* ct = new CtrlTask("pos", new TaskMap_Default(posTMT, getKinematics(), shape));
  ct->setTarget(pos);
  auto* path = createPathInJointSpace(ct, executionTime, verbose);
  delete ct;
  bool goodPath = false;
  if(path->isGood) {
    followQTrajectory(path);
    goodPath = true;
  }
  delete path;
  return goodPath;
}

bool Roopi::gotToJointConfiguration(const arr &jointConfig, double executionTime, bool verbose) {
  CtrlTask* ct = new CtrlTask("q", new TaskMap_qItself);
  ct->setTarget(jointConfig);
  Roopi_Path* path = createPathInJointSpace(ct, executionTime, verbose); //TODO why was there a auto*
  delete ct;
  bool goodPath = false;
  if(path->isGood) {
    followQTrajectory(path);
    goodPath = true;
  }
  delete path;
  return goodPath;
}

//==============================================================================

TaskReferenceInterpolAct::TaskReferenceInterpolAct(Roopi& roopi, const char* name, CtrlTask* task)
  : Thread(name, 0.01)
  , roopi(roopi)
  , task(task) {
  this->verbose = false;
}

TaskReferenceInterpolAct::~TaskReferenceInterpolAct() {
  threadClose();
}

void TaskReferenceInterpolAct::startExecution(double executionTime, const arr& reference, const arr& startState) {
  this->executionTime = executionTime;
  this->reference = reference;
  if(&startState) {
    initialRef = startState;
  } else {
    initialRef = roopi.getTaskValue(task);
  }
  startTime = mlr::timerRead();
  this->threadLoop();
}

void TaskReferenceInterpolAct::stopExecution() {
  this->threadStop();
}

void TaskReferenceInterpolAct::open() {}

void TaskReferenceInterpolAct::step() {
  double time = mlr::timerRead() - startTime;
  double s = time/executionTime;
  if(s > 1.0) {
    //cout << "finished" << endl;
    roopi.modifyCtrlTaskReference(task, reference);
    s = 1.0;
    this->state.setStatus(tsCLOSE); //TODO I have no glue if this is save :-)
  }
  arr actRef = initialRef + (reference - initialRef)*0.5*(1.0-cos(MLR_PI*s)); //TODO is this a good motion profile? Robotics lecture says yes :-)
  roopi.modifyCtrlTaskReference(task, actRef);
}

void TaskReferenceInterpolAct::close() {
  //delete this;
}

#endif


