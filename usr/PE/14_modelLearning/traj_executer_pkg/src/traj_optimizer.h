#ifndef TRAJ_OPTIMIZER_h
#define TRAJ_OPTIMIZER_h

#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>

struct TrajOptimizer {

  ors::KinematicWorld world;
  arr refPlan;
  double TRef;

  TrajOptimizer(ors::KinematicWorld &_world);
  void optimizeTrajectory(arr &_goal, arr &_q0, arr &x);
  void sampleGoal(arr &_goal, const arr &_q0);
};

#endif
