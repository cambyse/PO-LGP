#pragma once
#include "taskMap.h"

//===========================================================================

struct TaskMap_qLimits:TaskMap {
  arr limits;
  TaskMap_qLimits(const arr& _limits=NoArr){ if(&_limits) limits=_limits; } ///< if no limits are provided, they are taken from G's joints' attributes on the first call of phi
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

