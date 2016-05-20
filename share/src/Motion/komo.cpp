#include "komo.h"
#include "motion.h"
#include <Algo/spline.h>
#include <iomanip>
#include <Ors/ors_swift.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>

//===========================================================================

void setTasks(MotionProblem& MP,
              ors::Shape &endeff,
              ors::Shape& target,
              byte whichAxesToAlign,
              uint iterate,
              int timeSteps,
              double duration);

//===========================================================================

KOMO::KOMO() : MP(NULL){
}

KOMO::KOMO(const Graph& specs) : MP(NULL){
  init(specs);
//  reset();
//  CHECK(x.N,"");
}

void KOMO::init(const Graph& _specs){
  specs = _specs;

  Graph &glob = specs.get<Graph>("KOMO");
  uint timeStepsPerPhase=glob.get<double>("T");
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

  if(glob["makeSSBoxes"]){
    NIY;
    //for(ors::Shape *s: world.shapes) s->mesh.makeSSBox(s->mesh.V);
    world.gl().watch();
  }

  if(glob["activateAllContacts"])
    for(ors::Shape *s:world.shapes) s->cont=true;

  world.swift().initActivations(world);
  FILE("z.komo.model") <<world;

  if(MP) delete MP;
  MP = new MotionProblem(world);
  if(timeStepsPerPhase>=0) MP->setTiming(timeStepsPerPhase*phases, duration*phases);
  MP->k_order=k_order;

  MP->parseTasks(specs, timeStepsPerPhase);
}

void KOMO::setFact(const char* fact){
  specs.readNode(STRING(fact));
  MP->parseTask(specs.last());
}

void KOMO::setMoveTo(ors::KinematicWorld& world, ors::Shape& endeff, ors::Shape& target, byte whichAxesToAlign){
  if(MP) delete MP;
  MP = new MotionProblem(world);

  setTasks(*MP, endeff, target, whichAxesToAlign, 1, -1, -1.);
  reset();
}

void KOMO::setSpline(uint splineT){
  mlr::Spline S;
  S.setUniformNonperiodicBasis(MP->T, splineT, 2);
  uint n=MP->dim_x(0);
  splineB = zeros(S.basis.d0*n, S.basis.d1*n);
  for(uint i=0;i<S.basis.d0;i++) for(uint j=0;j<S.basis.d1;j++)
    splineB.setMatrixBlock(S.basis(i,j)*eye(n,n), i*n, j*n);
  z = pseudoInverse(splineB)* x;
}

void KOMO::reset(){
  x = MP->getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config
  if(splineB.N){
    z = pseudoInverse(splineB)* x;
  }

}

void KOMO::step(){
  NIY;
}

void KOMO::run(){
  ors::KinematicWorld::setJointStateCount=0;
//  cout <<x;
  if(MP->T){
    if(!splineB.N)
      optConstrained(x, dual, Convert(*MP), OPT(verbose=2));
    else{
      arr a,b,c,d,e;
      ConstrainedProblem P0 = conv_KOrderMarkovFunction2ConstrainedProblem(*MP);
      P0(a,b,c,NoTermTypeA, x);
      ConstrainedProblem P = conv_linearlyReparameterize(P0, splineB);
      P(a,b,NoArr,NoTermTypeA,z);
      optConstrained(z, dual, P, OPT(verbose=2));
    }
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
    if(!splineB.N)
      checkJacobianCP(Convert(*MP), x, 1e-4);
    else{
      ConstrainedProblem P0 = conv_KOrderMarkovFunction2ConstrainedProblem(*MP);
      ConstrainedProblem P = conv_linearlyReparameterize(P0, splineB);
      checkJacobianCP(P, z, 1e-4);
    }
  }else{
    checkJacobianCP(MP->InvKinProblem(), x, 1e-4);
  }
}

void KOMO::displayTrajectory(double delay){
#if 1
  MP->displayTrajectory(1, "KOMO planned trajectory", delay);
#else
  if(MP->T){
    ::displayTrajectory(x, 1, world, MP->switches, "KOMO planned trajectory", delay);
  //  orsDrawProxies=true;
  // for(uint t=0;t<x.d0;t++){
  //   MP->setState(x[t]);
  //   MP->world.gl().update(STRING("KOMO (time " <<std::setw(3) <<t <<'/' <<x.d0 <<')'));
  // }
  }else{
    world.setJointState(x);
    world.stepSwift();
    world.gl().watch("KOMO InvKin mode");
  }
  // if(wait) MP->world.gl().watch();
#endif
}

//===========================================================================

arr moveTo(ors::KinematicWorld& world,
           ors::Shape &endeff,
           ors::Shape& target,
           byte whichAxesToAlign,
           uint iterate,
           int timeSteps,
           double duration){

  MotionProblem MP(world);

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
      optConstrained(x, NoArr, Convert(MP), OPT(verbose=2)); //parameters are set in cfg!!
      //verbose=1, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
    }else{
      optNewton(x, Convert(MP), OPT(verbose=2));
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

  t = MP.addTask("transitions", new TransitionTaskMap(MP.world), sumOfSqrTT);
  if(timeSteps!=0){
    t->map.order=2; //make this an acceleration task!
  }else{
    t->map.order=1; //make this a velocity task!
  }
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  if(timeSteps!=0){
    t = MP.addTask("final_vel", new TaskMap_qItself(), sumOfSqrTT);
    t->map.order=1; //make this a velocity task!
    t->setCostSpecs(MP.T-4, MP.T, {0.}, zeroVelPrec);
  }

  if(colPrec<0){ //interpreted as hard constraint (default)
    t = MP.addTask("collisionConstraints", new CollisionConstraint(margin), ineqTT);
    t->setCostSpecs(0, MP.T, {0.}, 1.);
  }else{ //cost term
    t = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0u}, margin), sumOfSqrTT);
    t->setCostSpecs(0, MP.T, {0.}, colPrec);
  }

  t = MP.addTask("endeff_pos", new DefaultTaskMap(posTMT, endeff.index, NoVector, target.index, NoVector), sumOfSqrTT);
  t->setCostSpecs(MP.T, MP.T, {0.}, posPrec);


  for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
    ors::Vector axis;
    axis.setZero();
    axis(i)=1.;
    t = MP.addTask(STRING("endeff_align_"<<i),
                   new DefaultTaskMap(vecAlignTMT, endeff.index, axis, target.index, axis),
                   sumOfSqrTT);
    t->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);
  }
}

//===========================================================================

