#ifndef MF_STRATEGY_H
#define MF_STRATEGY_H

#include <Core/array.h>
#include <Kin/kin.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Algo/MLcourse.h>

#include "/usr/local/MATLAB/R2016a/extern/include/engine.h"
#define  BUFSIZE 1000

struct ConBOpt
{
  Engine *ep;
  arr X, Y, YS;
  uint nParam;
  arr pLimit;
  mlr::String folder;
  mlr::String name;
  arr x0;

  char buffer[BUFSIZE+1];

  double mu;
  ConBOpt(uint nParam_, arr &pLimit_, mlr::String folder_, mlr::String name_);
  ~ConBOpt();
  void addDatapoint(arr x,arr y, arr ys);
  void evaluate(arr &x);

  void save(int id=-1);
  void load(int id=-1);

  void sendArrToMatlab(arr &x, mlr::String name);
  void getArrFromMatlab(arr &x, mlr::String name);
};

#endif // MF_STRATEGY_H
