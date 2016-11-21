#ifndef TRAJ_OPTIMIZER_h
#define TRAJ_OPTIMIZER_h

#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>

struct TrajOptimizer {

  ors::KinematicWorld world;
  arr refPlan;
  double TRef;

  TrajOptimizer(ors::KinematicWorld &_world);
  void optimizeTrajectory(arr &_goal, arr &_q0);
};

#endif
