#pragma once

#include "taskMap.h"

//===========================================================================

/// defines a transition cost vector, which is q.N-dimensional and captures
/// accelerations or velocities over consecutive time steps
struct TaskMap_FixSwichedObjects:TaskMap {
  TaskMap_FixSwichedObjects(){}
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){ HALT("can only be of order 1"); }
  virtual uint dim_phi(const ors::KinematicWorld& G){ HALT("can only be of order 1"); }
  virtual uint dim_phi(const WorldL& G, int t);
  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return STRING("FixSwichedObjects"); }
};
