#pragma once

#include "taskMap.h"

struct TaskMap_qItself:TaskMap {
  arr M;            ///< optionally, the task map is M*q or M%q (linear in q)
  bool moduloTwoPi; ///< if false, consider multiple turns of a joint as different q values (Default: true)

  TaskMap_qItself(uint singleQ, uint qN); ///< The singleQ parameter generates a matrix M that picks out a single q value
  TaskMap_qItself(const arr& _M=NoArr);   ///< Specifying NoArr returns q; specifying a vector M returns M%q; specifying a matrix M returns M*q
  TaskMap_qItself(const ors::KinematicWorld& G, ors::Joint* j);
  TaskMap_qItself(const ors::KinematicWorld& G, const char* jointName);
  TaskMap_qItself(const ors::KinematicWorld& G, const char* jointName1, const char* jointName2);

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t);
  virtual uint dim_phi(const ors::KinematicWorld& G);
  virtual uint dim_phi(const WorldL& G, int t);
  virtual mlr::String shortTag(){ return STRING("qItself_" <<M.d0); }
private:
  uintA dimPhi;
};

//===========================================================================

struct TaskMap_qZeroVels:TaskMap {
  TaskMap_qZeroVels(){ }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){NIY}
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t);
  virtual uint dim_phi(const ors::KinematicWorld& G){NIY}
  virtual uint dim_phi(const WorldL& G, int t);
private:
  uintA dimPhi;
};
