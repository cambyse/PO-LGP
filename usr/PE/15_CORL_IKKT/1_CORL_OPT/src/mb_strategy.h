#ifndef MB_STRATEGY_H
#define MB_STRATEGY_H

#include <Core/array.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include "../../../src/traj_factory.h"
#include "../../../src/task_manager.h"

struct MB_strategy
{
  double duration;
  arr xDemo;
  MotionProblem* MP;
  ors::KinematicWorld* world;
  MotionProblemFunction* MPF;

  MB_strategy(arr &xDemo_, ors::KinematicWorld &world, double duration_, TaskManager &task);

  void evaluate(arr &X);
};

#endif // MB_STRATEGY_H
