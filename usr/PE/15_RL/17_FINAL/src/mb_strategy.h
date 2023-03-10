#ifndef MB_STRATEGY_H
#define MB_STRATEGY_H

#include <Core/array.h>
#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include "../../src/traj_factory.h"
#include "task_manager.h"

struct MB_strategy
{
  double duration;
  arr xDemo;
  KOMO* MP;
  mlr::KinematicWorld* world;
  MotionProblemFunction* MPF;

  MB_strategy(arr &xDemo_, mlr::KinematicWorld &world, double duration_, TaskManager &task);

  void evaluate(arr &X);
};

#endif // MB_STRATEGY_H
