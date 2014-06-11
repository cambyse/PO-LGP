#include "komo.h"
#include "motion.h"
#include <Ors/ors_swift.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>

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
  target.cont=false;

  MotionProblem MP(world);
  MP.loadTransitionParameters();
  world.swift().initActivations(world);

  TaskCost *c;
  c = MP.addTask("endeff_pos", new DefaultTaskMap(posTMT, endeff.index, NoVector, target.index, NoVector));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, {0.,0.,0.}, posPrec);

  c = MP.addTask("endeff_vel", new DefaultTaskMap(posTMT, world, "endeff")); //endeff.index));
//  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, world));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, zeroVelPrec);
  c->map.order=1; //make this a velocity variable!

  if(colPrec<0){ //interpreted as hard constraint
    c = MP.addTask("collisionConstraints", new CollisionConstraint(margin));
  }else{ //cost term
    c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, margin));
  }
  MP.setInterpolatingCosts(c, MotionProblem::constant, {0.}, colPrec);

  for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
    ors::Vector axis;
    axis.setZero();
    axis(i)=1.;
    c = MP.addTask(STRING("endeff_align_"<<i),
                   new DefaultTaskMap(vecAlignTMT, endeff.index, axis, target.index, axis));
    MP.setInterpolatingCosts(c, MotionProblem::finalOnly, {1.}, alignPrec);
  }

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = replicate(MP.x0, MP.T+1);
  rndGauss(x,.01,true); //don't initialize at a singular config

  //-- optimize
  for(uint k=0;k<iterate;k++){
    MT::timerStart();
    if(colPrec<0){
      optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
    }else{
      optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
    }
    cout <<"** optimization time=" <<MT::timerRead() <<endl;
//    checkJacobian(Convert(MF), x, 1e-5);
    MP.costReport();
  }

  return x;
}
