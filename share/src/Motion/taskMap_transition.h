#pragma once
#include "taskMap.h"

//===========================================================================

/// defines a transition cost vector, which is q.N-dimensional and captures
/// accelerations or velocities over consecutive time steps
struct TransitionTaskMap:TaskMap {
  double posCoeff, velCoeff, accCoeff;  ///< coefficients to blend between velocity and acceleration penalization
  arr H_rate_diag;            ///< cost rate (per TIME, not step), given as diagonal of the matrix H
  double H_rate;  ///< cost rate (per TIME, not step), given as scalar, will be multiplied by Joint->H (given in ors file)
  bool fixJointsOnly;
  TransitionTaskMap(const ors::KinematicWorld& G, bool fixJointsOnly=false);
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const ors::KinematicWorld& G){ return G.getJointStateDimension(); }
  virtual uint dim_phi(const WorldL& G, int t);
  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return STRING("TransitionTaskMap_fix"<<fixJointsOnly <<"_pos" <<posCoeff <<"_vel" <<velCoeff<<"_acc"<<accCoeff); }
};
