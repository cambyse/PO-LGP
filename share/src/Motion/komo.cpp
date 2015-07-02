#include "komo.h"
#include "motion.h"
#include <Ors/ors_swift.h>
#include <Motion/taskMaps.h>

//===========================================================================

KOMO::KOMO(const Graph& specs){
  init(specs);
  reset();
  CHECK(x.N,"");
}

void KOMO::init(const Graph& specs){
  Graph &glob = specs["KOMO"]->graph();
  MT::FileToken model = glob["model"]->V<MT::FileToken>();
  uint timeSteps=glob["T"]->V<double>();
  double duration=glob["duration"]->V<double>();

  world.read(model);
  world.meldFixedJoints();
  world.removeUselessBodies();
  makeConvexHulls(world.shapes);
  for(ors::Shape *s:world.shapes) s->cont=true;
  world.swift().initActivations(world);

  MP = new MotionProblem(world);
  MPF = new MotionProblemFunction(*MP);
  if(timeSteps>=0) MP->setTiming(timeSteps, duration);
  if(timeSteps==0) MP->k_order=1;

  NodeL tasks = specs.getNodes("Task");
  for(Node *t:tasks){
    Graph &T = t->graph();
    TaskMap *map = newTaskMap( T["map"]->graph(), world);
    Task *task = MP->addTask(t->keys.last(), map);
    map->order = T.V<double>("order", 0);
    MT::String type = T.V<MT::String>("type", STRING("sumOfSqr"));
    if(type=="sumOfSqr") map->type=sumOfSqrTT;
    else if(type=="inequal") map->type=ineqTT;
    else if(type=="equal") map->type=eqTT;
    else HALT("Task type must be sumOfSqr|ineq|eq");
    arr time = T.V<arr>("time",{0.,1.});
    task->setCostSpecs(time(0)*timeSteps, time(1)*timeSteps, T.V<arr>("target", {0.}), T.V<double>("scale", {100.}));
  }
}

void KOMO::reset(){
  x = replicate(MP->x0, MP->T+1); //we initialize with a constant trajectory!
  rndGauss(x,.01,true); //don't initialize at a singular config
}

void KOMO::step(){
  NIY;
}

void KOMO::run(){
  ors::KinematicWorld::setJointStateCount=0;
  optConstrainedMix(x, dual, Convert(*MPF), OPT(verbose=2)); //parameters are set in cfg!!
  cout <<"** optimization time=" <<MT::timerRead()
      <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
//    checkJacobian(Convert(MF), x, 1e-5);
  MP->costReport();
}

void KOMO::displayTrajectory(){
  ::displayTrajectory(x, 1, world, "KOMO planned trajectory", 0.01);
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
  arr x = replicate(MP.x0, MP.T+1); //we initialize with a constant trajectory!
  rndGauss(x,.01,true); //don't initialize at a singular config

  //-- optimize
  double colPrec = MT::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  ors::KinematicWorld::setJointStateCount=0;
  for(uint k=0;k<iterate;k++){
    MT::timerStart();
    if(colPrec<0){
      optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2)); //parameters are set in cfg!!
      //verbose=1, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
    }else{
      optNewton(x, Convert(MF), OPT(verbose=2, nonStrictSteps=(!k?15:5)));
    }
    cout <<"** optimization time=" <<MT::timerRead()
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
  double posPrec = MT::getParameter<double>("KOMO/moveTo/precision", 1e3);
  double colPrec = MT::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  double margin = MT::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
  double zeroVelPrec = MT::getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 1e1);
  double alignPrec = MT::getParameter<double>("KOMO/moveTo/alignPrecision", 1e3);

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


