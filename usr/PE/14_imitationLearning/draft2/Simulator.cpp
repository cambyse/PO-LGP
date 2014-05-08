#include "Simulator.h"

Demonstration execRun(Demonstration demo, InnerCostFunction* icf) {
  bool vis=false;

  MotionProblem MP(demo.world,false);
  MP.loadTransitionParameters();
  MP.setState(demo.q0,demo.q0*0.);

  MP.taskCosts = icf->taskCosts;
  MP.x0 = {0.,0.,0.,0.,0.};

  MotionProblemFunction F(MP);
  uint T=F.get_T(); uint k=F.get_k(); uint n=F.dim_x(); double dt = MP.tau;

  arr x(T+1,n); x.setZero();
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));

  if (vis) {
    cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<MP.world.getJointStateDimension()<<endl;
    MP.costReport(true);
    displayTrajectory(x, 1, MP.world, "planned trajectory", 0.01);
  }

  Demonstration res;
  res.qTraj = x;
  res.world = MP.world;
  res.q0 = demo.q0;
  return res;
}
