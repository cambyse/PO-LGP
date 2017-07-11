#ifndef TRAJ_OPTIMIZER_h
#define TRAJ_OPTIMIZER_h

#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

struct TrajOptimizer {

  mlr::KinematicWorld world;
  arr refPlan;
  double TRef;

  TrajOptimizer(mlr::KinematicWorld &_world);
  void optimizeTrajectory(arr &_goal, arr &_q0);
};

#endif
