/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#pragma once

#include "motion.h"

//===========================================================================

/// defines a transition cost vector, which is q.N-dimensional and captures
/// accelerations or velocities over consecutive time steps
struct TransitionTaskMap:TaskMap {
  double velCoeff, accCoeff;  ///< coefficients to blend between velocity and acceleration penalization
  arr H_rate_diag;            ///< cost rate (per TIME, not step), given as diagonal of the matrix H
  TransitionTaskMap(const ors::KinematicWorld& G);
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau);
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const ors::KinematicWorld& G){ return G.getJointStateDimension(); }
};

//===========================================================================

enum DefaultTaskMapType {
  posTMT,     ///< 3D position of reference
  vecTMT,     ///< 3D vec (orientation)
  quatTMT,    ///< 4D quaterion
  vecAlignTMT ///< 1D vector alignment, can have 2nd reference, param (optional) determins alternative reference world vector
};

struct DefaultTaskMap:TaskMap {
  DefaultTaskMapType type;
  int i, j;               ///< which shapes does it refer to?
  ors::Vector ivec, jvec; ///< additional position or vector

  DefaultTaskMap(DefaultTaskMapType type,
                 int iShape=-1, const ors::Vector& ivec=NoVector,
                 int jShape=-1, const ors::Vector& jvec=NoVector);

  DefaultTaskMap(DefaultTaskMapType type, const ors::KinematicWorld& G,
                 const char* iShapeName=NULL, const ors::Vector& ivec=NoVector,
                 const char* jShapeName=NULL, const ors::Vector& jvec=NoVector);

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G);
};

//===========================================================================

struct TaskMap_qItself:TaskMap {
  arr M;            ///< optionally, the task map is M*q or M%q (linear in q)
  TaskMap_qItself(uint singleQ, uint qN){ M=zeros(1,qN); M(0,singleQ)=1.; } ///< The singleQ parameter generates a matrix M that picks out a single q value
  TaskMap_qItself(const arr& _M=NoArr){ if(&_M) M=_M; }                     ///< Specifying NoArr returns q; specifying a vector M returns M%q; specifying a matrix M returns M*q
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G);
};

//===========================================================================

struct TaskMap_qLimits:TaskMap {
  arr limits;
  TaskMap_qLimits(const arr& _limits=NoArr){ if(&_limits) limits=_limits; } ///< if no limits are provided, they are taken from G's joints' attributes on the first call of phi
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

enum PTMtype {
  allPTMT, //phi=sum over all proxies (as is standard)
  listedVsListedPTMT, //phi=sum over all proxies between listed shapes
  allVersusListedPTMT, //phi=sum over all proxies between listed shapes
  allExceptListedPTMT, //as above, but excluding listed shapes
  bipartitePTMT, //sum over proxies between the two sets of shapes (shapes, shapes2)
  pairsPTMT, //sum over proxies of explicitly listed pairs (shapes is n-times-2)
  allExceptPairsPTMT, //sum excluding these pairs
  vectorPTMT //vector of all pair proxies (this is the only case where dim(phi)>1)
};

/// Proxy task variable
struct ProxyTaskMap:TaskMap {
  /// @name data fields
  PTMtype type;
  uintA shapes,shapes2;
  double margin;
  bool useCenterDist;
  bool useDistNotCost;

  ProxyTaskMap(PTMtype _type,
               uintA _shapes,
               double _margin=.02,
               bool _useCenterDist=true,
               bool _useDistNotCost=false);
  virtual ~ProxyTaskMap() {};
  
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G);
};

//===========================================================================

struct CollisionConstraint:TaskMap {
  double margin;
  CollisionConstraint(double _margin=.1):margin(_margin){ type=ineqTT; }
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct LimitsConstraint:TaskMap {
  double margin;
  arr limits;
  LimitsConstraint():margin(0.001){ type=ineqTT; }
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PairCollisionConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  double margin;
  PairCollisionConstraint(const ors::KinematicWorld& G, const char* iShapeName, const char* jShapeName,double _margin)
    : i(G.getShapeByName(iShapeName)->index),
      j(G.getShapeByName(jShapeName)->index),
      margin(_margin) {
    type=ineqTT;
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PlaneConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  arr planeParams;  ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  PlaneConstraint(const ors::KinematicWorld& G, const char* iShapeName, const arr& _planeParams)
    : i(G.getShapeByName(iShapeName)->index), planeParams(_planeParams){ type=ineqTT; }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

//this is NOT a constraint -- it turns a constraint into stickiness
struct ConstraintStickiness:TaskMap {
  TaskMap& map;
  ConstraintStickiness(TaskMap& _map)
    : map(_map) {
    type=sumOfSqrTT;
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PointEqualityConstraint:TaskMap {
  int i, j;               ///< which shapes does it refer to?
  ors::Vector ivec, jvec; ///< additional position or vector


  PointEqualityConstraint(const ors::KinematicWorld &G,
                          const char* iShapeName=NULL, const ors::Vector& _ivec=NoVector,
                          const char* jShapeName=NULL, const ors::Vector& _jvec=NoVector){
    DefaultTaskMap dummy(posTMT, G, iShapeName, _ivec, jShapeName, _jvec); //is deleted in a sec..
    i=dummy.i;
    j=dummy.j;
    ivec=dummy.ivec;
    jvec=dummy.jvec;
    type=eqTT;
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 3; }
};

struct ContactEqualityConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  double margin;
  ContactEqualityConstraint(const ors::KinematicWorld& G, const char* iShapeName, const char* jShapeName,double _margin)
    : i(G.getShapeByName(iShapeName)->index),
      j(G.getShapeByName(jShapeName)->index),
      margin(_margin) {
    type=eqTT;
  }
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){
    return 1;
  }
};

struct VelAlignConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  ors::Vector ivec, jvec; ///< additional position or vector
  double target;

  double margin;
  VelAlignConstraint(const ors::KinematicWorld& G,
                     const char* iShapeName=NULL, const ors::Vector& _ivec=NoVector,
                     const char* jShapeName=NULL, const ors::Vector& _jvec=NoVector, double _target = 0.);

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G) { } ;
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};


struct qItselfConstraint:TaskMap {
  arr M;

  qItselfConstraint(uint singleQ, uint qN){ M=zeros(1,qN); M(0,singleQ)=1.; type=eqTT; }
  qItselfConstraint(const arr& _M=NoArr){ if(&_M) M=_M; type=eqTT;}

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){
    if(M.nd==2) return M.d0;
    return G.getJointStateDimension();
  }
};
