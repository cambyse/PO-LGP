#ifndef TRAJ_OPTIMIZER_h
#define TRAJ_OPTIMIZER_h

#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>

struct TrajOptimizer {

  mlr::KinematicWorld world;
  arr refPlan;
  double TRef;

  TrajOptimizer(mlr::KinematicWorld &_world);
  void optimizeTrajectory(arr &_goal, arr &_q0);
};

#endif
