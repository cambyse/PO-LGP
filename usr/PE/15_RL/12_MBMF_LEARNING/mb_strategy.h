#ifndef MB_STRATEGY_H
#define MB_STRATEGY_H

#include <Core/array.h>
#include <Kin/kin.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include "task_manager.h"

struct MB_strategy
{
  arr xDemo;
  MotionProblem* MP;
  mlr::KinematicWorld* world;
  MotionProblemFunction* MPF;

  MB_strategy(arr &xDemo_, mlr::KinematicWorld &world, TaskManager &tm);

  void evaluate(arr &X);
};

#endif // MB_STRATEGY_H
