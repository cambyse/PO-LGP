#include "poseGenerator.h"

#include <Algo/MLcourse.h>

#include <Motion/komo.h>
#include <Motion/motion.h>

PoseGenerator::PoseGenerator(const ors::KinematicWorld& W) : W(W) {
  q0 = W.getJointState();
}

arr PoseGenerator::getRandomPose(const arr& qInit) {
  if(&qInit) {
    W.setJointState(qInit);
  } else {
    W.setJointState(q0);
  }

  // sample random position and orientation pose
  arr posR = .3*randn(3);  posR += ARR(.6, -.3, 1.);
  arr posL = .3*randn(3);  posL += ARR(.6,  .3, 1.);
  arr vecR = randn(3); /*if(vecR(0)<0.) vecR(0) *=-1.;*/  vecR/=length(vecR);
  arr vecL = randn(3); /*if(vecL(0)<0.) vecL(0) *=-1.;*/  vecL/=length(vecL);

  KOMO komo;
  komo.setModel(W);
  komo.setTiming(1, 1, 5., 1, true);
  komo.setSquaredQVelocities();
  komo.setCollisions(true);
  komo.setLimits(true, 0.1);
  komo.setPosition(1., 1., "endeffR", NULL, sumOfSqrTT, posR);
  komo.setPosition(1., 1., "endeffL", NULL, sumOfSqrTT, posL);
  komo.setAlign(1., 1., "endeffR", ARR(1.,0.,0.), NULL, vecR, sumOfSqrTT, {1.});
  komo.setAlign(1., 1., "endeffL", ARR(1.,0.,0.), NULL, vecL, sumOfSqrTT, {1.});

  arr lim = W.getLimits();
  uint qIdx = W.getJointByName("head_tilt_joint")->qIndex;
  double qh = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0));
  Task* t = komo.MP->addTask("headTilt", new TaskMap_qItself(qIdx,W.getJointStateDimension()), sumOfSqrTT);
  t->setCostSpecs(komo.MP->T-1, komo.MP->T, ARR(qh), 1.);

  komo.reset();
  komo.run();

  Graph result = komo.getReport();
  //    cout <<result <<endl;
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if(constraints<.1 && cost<5.){
    komo.x.refRange(0,2)=0.;
    W.setJointState(komo.x);
    W.watch(false);
    return komo.x;
  }else{
    return getRandomPose(qInit);
  }
}

#if 0
arr PoseGenerator::getTraj(arr qInit, arr qEnd) {
  MotionProblem MP(W);
  Task *t;
  MP.world.setJointState(qInit);

  t = MP.addTask("transitions", new TaskMap_Transition(W), sumOfSqrTT);
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("q", new TaskMap_qItself, sumOfSqrTT);
  t->setCostSpecs(MP.T-1, MP.T, qEnd, 1e2);

  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1), ineqTT);
  t->setCostSpecs(0, MP.T, {0.}, 1.);
  t = MP.addTask("limits", new LimitsConstraint(0.1), ineqTT);
  t->setCostSpecs(10, MP.T, {0.}, 1.);

  //    // sample a head joint
  //    arr lim = W.getLimits();
  //    uint qIdx = W.getJointByName("head_pan_joint")->qIndex;
  //    double qh = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0));
  //    t = MP.addTask("head1", new TaskMap_qItself(qIdx,W.getJointStateDimension()), sumOfSqrTT);
  //    t->setCostSpecs(MP.T-1, MP.T, ARR(qh), 1.);
  //    qIdx = W.getJointByName("head_tilt_joint")->qIndex;
  //    qh = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0));
  //    t = MP.addTask("head2", new TaskMap_qItself(qIdx,W.getJointStateDimension()), sumOfSqrTT);
  //    t->setCostSpecs(MP.T-1, MP.T, ARR(qh), 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  arr x = MP.getInitialization(); //replicate(MP.x0, MP.T+1);
  x.reshape(MP.T,W.getJointStateDimension());

  //-- optimize
  optConstrained(x, NoArr, Convert(MP), OPT(verbose=1, stopIters=100, stopTolerance=1e-3, damping=1., maxStep=1.,aulaMuInc=2, nonStrictSteps=5));
  cout <<"** optimization time=" <<mlr::timerRead()
      <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
  MP.costReport();

  Graph result = MP.getReport();
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  W.setJointState(x[x.d0-1]);
  TaskMap_qLimits limits;
  arr y;
  limits.phi(y,NoArr,W);
  cout <<"Limits: " <<y<< endl;
  //displayTrajectory(x, 1, W, "planned trajectory", 0.01);
  if(constraints<.1 && cost<5.){
    W.setJointState(x[x.d0-1]);
    //W.gl().update();
    gnuplot("load 'z.costReport.plt'", false, true);
    //      displayTrajectory(x, 1, W, "planned trajectory", 0.01);
    return x;
  } else {
    //return getTraj(qInit, getPose());
  }
}
#endif
