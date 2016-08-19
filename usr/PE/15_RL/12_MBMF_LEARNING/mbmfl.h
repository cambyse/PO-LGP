#ifndef MBMFL_H
#define MBMFL_H

#include <Core/array.h>
#include <Motion/motion.h>

struct MBMFL
{
  mlr::String name;
  mlr::Array<arr> data_traj;  // collected trajectories
  arr data_param; // collected parametrizations
  arr data_result;           // collected results
  arr data_cost;

  MBMFL(mlr::String name_);

  void saveMBMFL();
  void loadMBMFL();

  void addDatapoint(arr X, arr param, bool Y, double c);
};

#endif // MBMFL_H
