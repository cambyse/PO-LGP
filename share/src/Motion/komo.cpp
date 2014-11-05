#include "komo.h"
#include "motion.h"
#include <Ors/ors_swift.h>
#include <Motion/taskMaps.h>

arr moveTo(ors::KinematicWorld& world,
           ors::Shape &endeff,
           ors::Shape& target,
           byte whichAxesToAlign,
           uint iterate){
  //-- parameters
  double posPrec = MT::getParameter<double>("KOMO/moveTo/precision", 1e3);
  double colPrec = MT::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
  double margin = MT::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
  double zeroVelPrec = MT::getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 1e1);
  double alignPrec = MT::getParameter<double>("KOMO/moveTo/alignPrecision", 1e3);

  //-- set up the MotionProblem
  target.cont=false; //turn off contact penalization with the target

  MotionProblem MP(world);
  world.swift().initActivations(world);
  MP.world.watch(false);

  Task *c;

  c = MP.addTask("transitions", new TransitionTaskMap(world));
  c->map.order=2; //make this an acceleration task!
  c->setCostSpecs(0, MP.T, {0.}, 1e0);

  c = MP.addTask("endeff_pos", new DefaultTaskMap(posTMT, endeff.index, NoVector, target.index, NoVector));
  c->setCostSpecs(MP.T, MP.T, {0.}, posPrec);

  c = MP.addTask("endeff_vel", new DefaultTaskMap(posTMT, world, "endeff"));
//  c = MP.addTask("q_vel", new TaskMap_qItself());
  c->setCostSpecs(MP.T, MP.T, {0.}, zeroVelPrec);
  c->map.order=1; //make this a velocity task!

  if(colPrec<0){ //interpreted as hard constraint (default)
    c = MP.addTask("collisionConstraints", new CollisionConstraint(margin));
  }else{ //cost term
    c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, margin));
  }
  c->setCostSpecs(0, MP.T, {0.}, colPrec);

  for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
    ors::Vector axis;
    axis.setZero();
    axis(i)=1.;
    c = MP.addTask(STRING("endeff_align_"<<i),
                   new DefaultTaskMap(vecAlignTMT, endeff.index, axis, target.index, axis));
    c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);
  }

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = replicate(MP.x0, MP.T+1); //we initialize with a constant trajectory!
  rndGauss(x,.01,true); //don't initialize at a singular config

  //-- optimize
  ors::KinematicWorld::setJointStateCount=0;
  for(uint k=0;k<iterate;k++){
    MT::timerStart();
    if(colPrec<0){
      optConstrained(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false));
      //verbose=1, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
    }else{
      optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
    }
    cout <<"** optimization time=" <<MT::timerRead()
        <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
//    checkJacobian(Convert(MF), x, 1e-5);
    MP.costReport();
  }

  return x;
}
