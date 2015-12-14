#include "komo.h"
#include "motion.h"
#include <iomanip>
#include <Ors/ors_swift.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>

//===========================================================================

KOMO::KOMO(const Graph& specs){
  init(specs);
//  reset();
//  CHECK(x.N,"");
}

void KOMO::init(const Graph& _specs){
  specs = _specs;

  Graph &glob = specs.get<Graph>("KOMO");
  uint timeSteps=glob.get<double>("T");
  double duration=glob.get<double>("duration");
  uint phases=glob.get<double>("phases", 1);
  uint k_order=glob.get<double>("k_order", 2);

  if(glob["model"]){
    mlr::FileToken model = glob.get<mlr::FileToken>("model");
    world.read(model);
  }else{
    world.init(specs);
  }

  if(glob["meldFixedJoints"]){
    world.meldFixedJoints();
    world.removeUselessBodies();
  }

  if(glob["makeConvexHulls"])
    makeConvexHulls(world.shapes);

  if(glob["activateAllContacts"])
    for(ors::Shape *s:world.shapes) s->cont=true;

  world.swift().initActivations(world);
  FILE("z.komo.model") <<world;

  MP = new MotionProblem(world);
  if(timeSteps>=0) MP->setTiming(timeSteps*phases, duration*phases);
  MP->k_order=k_order;

#if 1
  MP->parseTasks(specs);
  if(MP->T) MPF = new MotionProblemFunction(*MP);
#else
  NodeL tasks = specs.getNodes("Task");
  for(Node *t:tasks){
    Graph &T = t->graph();
    TaskMap *map = newTaskMap( T["map"]->graph(), world);
    Task *task = MP->addTask(t->keys.last(), map);
    map->order = T.V<double>("order", 0);
    mlr::String type = T.V<mlr::String>("type", STRING("sumOfSqr"));
    if(type=="sumOfSqr") map->type=sumOfSqrTT;
    else if(type=="inequal") map->type=ineqTT;
    else if(type=="equal") map->type=eqTT;
    else HALT("Task type must be sumOfSqr|ineq|eq");
    arr time = T.V<arr>("time",{0.,1.});
    task->setCostSpecs(time(0)*timeSteps, time(1)*timeSteps, T.V<arr>("target", {0.}), T.V<double>("scale", {100.}));
  }

  NodeL switches = specs.getNodes("KinematicSwitch");
  for(Node *s:switches){
    Graph &S = s->graph();
    ors::KinematicSwitch *sw= new ors::KinematicSwitch();
    mlr::String type = S.get<mlr::String>("type");
    if(type=="addRigid"){ sw->symbol=ors::KinematicSwitch::addJointZero; sw->jointType=ors::JT_fixed; }
    else if(type=="addRigidRel"){ sw->symbol = ors::KinematicSwitch::addJointAtTo; sw->jointType=ors::JT_fixed; }
    else if(type=="transXYPhi"){ sw->symbol = ors::KinematicSwitch::addJointAtFrom; sw->jointType=ors::JT_transXYPhi; }
    else if(type=="free"){ sw->symbol = ors::KinematicSwitch::addJointAtTo; sw->jointType=ors::JT_free; }
    else if(type=="delete"){ sw->symbol = ors::KinematicSwitch::deleteJoint; }
    sw->timeOfApplication = S.get<double>("timeOfApplication")*timeSteps+1;
    sw->fromId = world.getShapeByName(S.get<mlr::String>("from"))->index;
    sw->toId = world.getShapeByName(S.get<mlr::String>("to"))->index;
    MP->switches.append(sw);
  }

  if(MP->T){
    MPF = new MotionProblemFunction(*MP);
  }else{
    LOG(0) <<"InvKin mode";
    TaskMap *map = new TaskMap_qItself();
    Task *task = MP->addTask("InvKinTransition", map);
    map->order = 0;
    map->type=sumOfSqrTT;
    task->setCostSpecs(0, 0, MP->x0, 1./(MP->tau*MP->tau));
  }
#endif

#if 0
  reset();
  arr y,J;
  TermTypeA tt;
  MP->inverseKinematics(y,J,tt,x);
  MP->reportFull();
#endif
}

void KOMO::setFact(const char* fact){
  specs.readNode(STRING(fact));
  MP->parseTask(specs.last());
}

void KOMO::reset(){
  if(MP->T){
    x = MP->getInitialization();
  }else{
    x=MP->x0;
  }
  rndGauss(x,.01,true); //don't initialize at a singular config
}

void KOMO::step(){
  NIY;
}

void KOMO::run(){
  ors::KinematicWorld::setJointStateCount=0;
  cout <<x;
  if(MP->T){
    optConstrained(x, dual, Convert(*MPF), OPT(verbose=2));
  }else{
    optConstrained(x, dual, MP->InvKinProblem(), OPT(verbose=2));
  }
  cout <<"** optimization time=" <<mlr::timerRead()
      <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
  //    checkJacobian(Convert(MF), x, 1e-5);
  MP->costReport(false);
}

Graph KOMO::getReport(){
  return MP->getReport();
}

void KOMO::checkGradients(){
  if(MP->T){
    checkJacobianCP(Convert(*MPF), x, 1e-4);
  }else{
    checkJacobianCP(MP->InvKinProblem(), x, 1e-4);
  }
}

void KOMO::displayTrajectory(double delay){
  if(MP->T){
    ::displayTrajectory(x, 1, world, MP->switches, "KOMO planned trajectory", delay);
  //  orsDrawProxies=true;
  // for(uint t=0;t<x.d0;t++){
  //   MP->setState(x[t]);
  //   MP->world.gl().update(STRING("KOMO (time " <<std::setw(3) <<t <<'/' <<x.d0 <<')'));
  // }
  }else{
    MP->setState(x);
    MP->world.gl().watch("KOMO InvKin mode");
  }
  // if(wait) MP->world.gl().watch();
}

//===========================================================================

void setTasks(MotionProblem& MP,
              ors::Shape &endeff,
              ors::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration);

arr moveTo(ors::KinematicWorld& world,
           ors::Shape &endeff,
           ors::Shape& target,
           byte whichAxesToAlign,
           uint iterate,
           int timeSteps,
           double duration){

  MotionProblem MP(world);
  MotionProblemFunction MF(MP);

  setTasks(MP, endeff, target, whichAxesToAlign, iterate, timeSteps, duration);

  //-- create the Optimization problem (of type kOrderMarkov)
  arr x = MP.getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config

  //-- optimize
  double colPrec = mlr::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  ors::KinematicWorld::setJointStateCount=0;
  for(uint k=0;k<iterate;k++){
    mlr::timerStart();
    if(colPrec<0){
      optConstrained(x, NoArr, Convert(MF), OPT(verbose=2)); //parameters are set in cfg!!
      //verbose=1, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
    }else{
      optNewton(x, Convert(MF), OPT(verbose=2, nonStrictSteps=(!k?15:5)));
    }
    cout <<"** optimization time=" <<mlr::timerRead()
        <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
    //    checkJacobian(Convert(MF), x, 1e-5);
    MP.costReport();
  }

  return x;
}

void setTasks(MotionProblem& MP,
              ors::Shape &endeff,
              ors::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration){

  //-- parameters
  double posPrec = mlr::getParameter<double>("KOMO/moveTo/precision", 1e3);
  double colPrec = mlr::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  double margin = mlr::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
  double zeroVelPrec = mlr::getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 1e1);
  double alignPrec = mlr::getParameter<double>("KOMO/moveTo/alignPrecision", 1e3);

  //-- set up the MotionProblem
  target.cont=false; //turn off contact penalization with the target

  MP.world.swift().initActivations(MP.world);
  //MP.world.watch(false);

  if(timeSteps>=0) MP.setTiming(timeSteps, duration);
  if(timeSteps==0) MP.k_order=1;

  Task *t;

  t = MP.addTask("transitions", new TransitionTaskMap(MP.world));
  if(timeSteps!=0){
    t->map.order=2; //make this an acceleration task!
  }else{
    t->map.order=1; //make this a velocity task!
  }
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  if(timeSteps!=0){
    t = MP.addTask("final_vel", new TaskMap_qItself());
    t->map.order=1; //make this a velocity task!
    t->setCostSpecs(MP.T-4, MP.T, {0.}, zeroVelPrec);
  }

  if(colPrec<0){ //interpreted as hard constraint (default)
    t = MP.addTask("collisionConstraints", new CollisionConstraint(margin));
    t->setCostSpecs(0, MP.T, {0.}, 1.);
  }else{ //cost term
    t = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0u}, margin));
    t->setCostSpecs(0, MP.T, {0.}, colPrec);
  }

  t = MP.addTask("endeff_pos", new DefaultTaskMap(posTMT, endeff.index, NoVector, target.index, NoVector));
  t->setCostSpecs(MP.T, MP.T, {0.}, posPrec);


  for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
    ors::Vector axis;
    axis.setZero();
    axis(i)=1.;
    t = MP.addTask(STRING("endeff_align_"<<i),
                   new DefaultTaskMap(vecAlignTMT, endeff.index, axis, target.index, axis));
    t->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);
  }
}

//===========================================================================

