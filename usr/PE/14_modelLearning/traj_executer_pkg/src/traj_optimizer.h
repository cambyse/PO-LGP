#ifndef TRAJ_OPTIMIZER_h
#define TRAJ_OPTIMIZER_h

#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
enum BM_TYPE {CIRCLE, EIGHT, STAR};

struct TrajOptimizer {

  ors::KinematicWorld world;
  arr refPlan;
  arr traj;
  arr limits;
  double dt;
  TrajOptimizer(ors::KinematicWorld &_world);
  void optimizeTrajectory(arr &_goal, arr &_q0, arr &x);
  void sampleGoal(arr &_goal, const arr &_q0);

  void optimizeBenchmarkMotion(BM_TYPE type,arr &_q0, arr &x);
};

#endif
