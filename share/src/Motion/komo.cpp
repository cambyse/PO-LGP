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

  Task *t;

  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

//  t = MP.addTask("final_vel", new TransitionTaskMap(world));
  t = MP.addTask("final_vel", new TaskMap_qItself());
  t->map.order=1; //make this a velocity task!
  t->setCostSpecs(MP.T-4, MP.T, {0.}, zeroVelPrec);

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

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = replicate(MP.x0, MP.T+1); //we initialize with a constant trajectory!
  rndGauss(x,.01,true); //don't initialize at a singular config

  //-- optimize
  ors::KinematicWorld::setJointStateCount=0;
  for(uint k=0;k<iterate;k++){
    MT::timerStart();
    if(colPrec<0){
      optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2, aulaMuInc=1.1));
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
