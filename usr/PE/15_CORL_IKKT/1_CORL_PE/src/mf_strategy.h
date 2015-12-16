#ifndef MF_STRATEGY_H
#define MF_STRATEGY_H

#include <Core/array.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Algo/MLcourse.h>

#include "/usr/local/MATLAB/R2015a/extern/include/engine.h"
#define  BUFSIZE 512


struct MF_strategy
{
  Engine *ep;
  arr X, Y, YS;
  uint nParam;
  arr paramLimit;
  char buffer[BUFSIZE+1];

  double mu;
  MF_strategy(uint nParam_, arr &paramLimit_, mlr::String folder, mlr::String taskName);
  ~MF_strategy();
  void addDatapoint(arr x,arr y, arr ys);
  void evaluate(arr &x_exp);

  void save(mlr::String folder);
  void load(mlr::String folder);
};

#endif // MF_STRATEGY_H
