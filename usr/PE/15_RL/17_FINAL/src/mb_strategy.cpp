#include "mb_strategy.h"

#include <Kin/kin.h>

MB_strategy::MB_strategy(arr &xDemo_,mlr::KinematicWorld &world_,double duration_,TaskManager &task)
{
  xDemo = xDemo_;
  world = new mlr::KinematicWorld(world_);

  duration = duration_;
  MP = new KOMO(*world,false);

  MP->T = xDemo.d0-1;
  MP->tau =duration/MP->T;
  MP->x0 = xDemo[0];

  Task *t;
  t = MP->addTask("tra", new TransitionTaskMap(*world));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = world->getHmetric();
  t->map.order=2;
  t->setCostSpecs(0, MP->T, ARR(0.), 1e-1);

  // final position constraint
  t = MP->addTask("qT", new TaskMap_qItself());
  t->setCostSpecs(MP->T,MP->T,xDemo[xDemo.d0-1],1e2);

  // contact constraint
  task.addConstraints(MP,xDemo);

  MPF = new MotionProblemFunction(*MP);
}

void MB_strategy::evaluate(arr &X)
{
  arr tmp = X;
  OptOptions o; o.maxStep = mlr::getParameter<double>("MB_maxStep");
  o.stopTolerance = 1e-5; o.constrainedMethod=anyTimeAula; o.verbose=1;
  ConstrainedProblem CPM = Convert(*MPF);
  LagrangianProblem UPM(CPM, o.constrainedMethod);

  OptNewton opt(X, UPM, o);
  opt.step();
  cout << "MB - change of trajectory: " << sumOfSqr(tmp-X) << endl;
}
