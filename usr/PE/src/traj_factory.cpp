#include "traj_factory.h"

void TrajFactory::transform(const arr &y, const arr &trans, arr &y_trans, double s_mu, double s_std) {
  CHECK(trans.d0==y.d1,"dimension mismatch");

  // compute gaussian weight
  arr s_eval = linspace(0.,1.,y.d0-1).flatten();
  s_mu = s_eval((fabs(s_eval-s_mu)).minIndex());
  arr s = exp(-(s_eval-s_mu)%(s_eval-s_mu)/(2*pow(s_std,2)));
  y_trans = y + repmat(s,1,y.d1)%repmat(~trans,y.d0,1);
}


void TrajFactory::compFeatTraj(const arr &x, arr &y, ors::KinematicWorld &world, TaskMap *tm) {
  y.resize(x.d0,tm->dim_phi(world)).setZero();
  for (uint i=0;i<y.d0;i++) {
    arr y_t;
    world.setJointState(x[i]);
    tm->phi(y_t,NoArr,world);
    y[i] = y_t;
  }
}

/// transform a trajectory from feature space into joint space
void TrajFactory::compJointTraj(const arr &xInit, const arr &y, arr &x, MotionProblem &MP, TaskMap *tm) {
  MP.tasks.clear();
  MP.setState(xInit[0]);
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(MP.world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  t =  MP.addTask("t",tm);
  t->target = y;
  t->prec = ones(MP.T+1)*1e2;

  MotionProblemFunction MPF(MP);
  x = xInit;
  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));
}
