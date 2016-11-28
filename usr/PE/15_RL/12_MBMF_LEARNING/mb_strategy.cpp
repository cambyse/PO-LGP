#include "mb_strategy.h"
#include <Motion/pr2_heuristics.h>

MB_strategy::MB_strategy(arr &xDemo_,mlr::KinematicWorld &world_,TaskManager &tm):
  xDemo(xDemo_)
{
  world = new mlr::KinematicWorld(world_);
  MP = new MotionProblem(*world,false);

  MP->T = xDemo.d0-1;
  MP->tau = 0.02;
  MP->x0 = xDemo[0];

  Task *t;
  t = MP->addTask("tra", new TransitionTaskMap(*world));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(*world);
  t->map.order=2;

  t->setCostSpecs(0, MP->T, ARR(0.), 1e-2);

  // final position constraint
  t = MP->addTask("qT", new TaskMap_qItself());
  t->setCostSpecs(MP->T,MP->T,xDemo[xDemo.d0-1],1e2);

  // endeffector position constraint
  tm.addConstraintTaskMaps(*MP,ARR(0.0));//zeros(1));

  MPF = new MotionProblemFunction(*MP);
}

void MB_strategy::evaluate(arr &X)
{
  arr tmp = X;
  OptOptions o; o.maxStep = mlr::getParameter<double>("MB_maxStep");
  o.stopTolerance = 1e-5; o.constrainedMethod=anyTimeAula; o.verbose=1;
  ConstrainedProblemMix CPM = Convert(*MPF);
  LagrangianProblemMix UPM(CPM, o.constrainedMethod);
  OptNewton opt(X, UPM, o);

  opt.step();
  cout << "MB - change of trajectory: " << sumOfSqr(tmp-X) << endl;
}
