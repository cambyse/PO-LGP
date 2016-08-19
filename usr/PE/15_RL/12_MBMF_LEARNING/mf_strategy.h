#ifndef MF_STRATEGY_H
#define MF_STRATEGY_H

#include <Core/array.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Algo/MLcourse.h>
#include "task_manager.h"

struct MF_strategy
{  
  arr X_grid, F_grid;
  double mu;
  MF_strategy(double paramDim,TaskManager &tm);
  void evaluate(arr &Xnext, double &Rnext, arr &data_param, arr &data_result, arr &data_cost);
  arr improvementFun(const arr &F, const arr &F_grid);
  arr acquisitionFun(const arr &F,const arr &F_grid,const arr &F_std,const arr &P_grid);
  void recomputeCosts(TaskManager &tm, arr &X);
};

#endif // MF_STRATEGY_H
