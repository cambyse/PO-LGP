#pragma once

#include "taskMap.h"

struct TaskMap_BeliefTransition : TaskMap{
  TaskMap_BeliefTransition(){}
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const mlr::KinematicWorld& G);
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("BeliefTransition"); }

};
