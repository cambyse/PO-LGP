#ifndef MPC_H
#define MPC_H

#include <Core/util.h>
#include <Core/array.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include "../splines/spline.h"


struct MPC {
  MPC(uint _plan_time_factor, ors::Graph &_orsG);
  ~MPC();

  arr iterate(double _t, arr &_state, arr &_goal, double _simRate);
  void replanTrajectory(arr& _state,arr& _goal, double _t);

  double control_time;
  double plan_time;
  uint plan_time_factor;

  MotionProblem *P;
  MotionProblemFunction *F;
  ors::Graph *orsG;

  Spline *s;

  arr y;
  arr yRef;

  uint T,n;
  double dt;
  double t_prev;
  double t_plan_prev;

  // bookkeeping
  arr y_bk;

  arr y_cart;

};

#endif // MPC_H
