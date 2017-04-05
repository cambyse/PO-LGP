#ifndef MPC_H
#define MPC_H

#include <Core/util.h>
#include <Core/array.h>
#include <Kin/kin.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Algo/spline.h>


struct MPC {
  MPC(MotionProblem &_P, arr &_x);
  ~MPC();


  void replan(arr& _goal, arr &_q);

  MotionProblem& P;
  arr x;

  // bookkeeping
  arr x_bk;
  arr x_cart;

};

#endif // MPC_H
