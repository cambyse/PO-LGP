/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include "taskMap.h"

#include "taskMap_qItself.h"
#include "taskMap_GJK.h"
#include "taskMap_transition.h"
#include "taskMap_default.h"
#include "taskMap_qLimits.h"
#include "taskMap_pushConsistent.h"
#include "taskMap_FixAttachedObjects.h"
#include "taskMap_AboveBox.h"
#include "taskMap_AlignStacking.h"

//===========================================================================

enum PTMtype {
  allPTMT, //phi=sum over all proxies (as is standard)
  listedVsListedPTMT, //phi=sum over all proxies between listed shapes
  allVsListedPTMT, //phi=sum over all proxies against listed shapes
  allExceptListedPTMT, //as above, but excluding listed shapes
  bipartitePTMT, //sum over proxies between the two sets of shapes (shapes, shapes2)
  pairsPTMT, //sum over proxies of explicitly listed pairs (shapes is n-times-2)
  allExceptPairsPTMT, //sum excluding these pairs
  vectorPTMT //vector of all pair proxies (this is the only case where dim(phi)>1)
};

//===========================================================================

/// Proxy task variable
struct TaskMap_Proxy:TaskMap {
  /// @name data fields
  PTMtype type;
  uintA shapes,shapes2;
  double margin;
  bool useCenterDist;
  bool useDistNotCost;

  TaskMap_Proxy(PTMtype _type,
               uintA _shapes,
               double _margin=.02,
               bool _useCenterDist=false,
               bool _useDistNotCost=false);
  virtual ~TaskMap_Proxy() {}
  
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G);
};

//===========================================================================

struct CollisionConstraint:TaskMap {
  double margin;
  CollisionConstraint(double _margin=.1):margin(_margin){}
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("CollisionConstraint"); }
};


//===========================================================================

struct ProxyConstraint:TaskMap {
  TaskMap_Proxy proxyCosts;
  ProxyConstraint(PTMtype _type,
                  uintA _shapes,
                  double _margin=.02,
                  bool _useCenterDist=false,
                  bool _useDistNotCost=false);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return "ProxyConstraint"; }
};

//===========================================================================

struct LimitsConstraint:TaskMap {
  double margin;
  arr limits;
  LimitsConstraint(double _margin=.05):margin(_margin){}
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("LimitsConstraint"); }
};

//===========================================================================

struct PairCollisionConstraint:TaskMap {
  int i,j;       ///< which shapes does it refer to?
  double margin;
  intA referenceIds; ///< the shapes it refers to DEPENDENT on time
  PairCollisionConstraint(double _margin)
    : i(-1), j(-1), margin(_margin){
  }
  PairCollisionConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName, double _margin=.02)
    : i(G.getShapeByName(iShapeName)->index),
      j(G.getShapeByName(jShapeName)->index),
      margin(_margin) {
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PlaneConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  arr planeParams;  ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  PlaneConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const arr& _planeParams)
    : i(G.getShapeByName(iShapeName)->index), planeParams(_planeParams){}

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
};

//===========================================================================

//this is NOT a constraint -- it turns a constraint into stickiness
struct ConstraintStickiness:TaskMap {
  TaskMap& map;
  ConstraintStickiness(TaskMap& _map)
    : map(_map) {
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PointEqualityConstraint:TaskMap {
  int i, j;               ///< which shapes does it refer to?
  mlr::Vector ivec, jvec; ///< additional position or vector

  PointEqualityConstraint(const mlr::KinematicWorld &G,
                          const char* iShapeName=NULL, const mlr::Vector& _ivec=NoVector,
                          const char* jShapeName=NULL, const mlr::Vector& _jvec=NoVector){
    TaskMap_Default dummy(posTMT, G, iShapeName, _ivec, jShapeName, _jvec); //is deleted in a sec..
    i=dummy.i;
    j=dummy.j;
    ivec=dummy.ivec;
    jvec=dummy.jvec;
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 3; }
};

//===========================================================================

struct ContactEqualityConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  double margin;
  ContactEqualityConstraint(const mlr::KinematicWorld& G, const char* iShapeName, const char* jShapeName,double _margin)
    : i(G.getShapeByName(iShapeName)->index),
      j(G.getShapeByName(jShapeName)->index),
      margin(_margin) {
  }
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){
    return 1;
  }
};

//===========================================================================

struct VelAlignConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  mlr::Vector ivec, jvec; ///< additional position or vector
  double target;

  double margin;
  VelAlignConstraint(const mlr::KinematicWorld& G,
                     const char* iShapeName=NULL, const mlr::Vector& _ivec=NoVector,
                     const char* jShapeName=NULL, const mlr::Vector& _jvec=NoVector, double _target = 0.);

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1) { } ;
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct qItselfConstraint:TaskMap {
  arr M;

  qItselfConstraint(uint singleQ, uint qN){ M=zeros(1,qN); M(0,singleQ)=1.; }
  qItselfConstraint(const arr& _M=NoArr){ if(&_M) M=_M; }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const mlr::KinematicWorld& G){
    if(M.nd==2) return M.d0;
    return G.getJointStateDimension();
  }
};

//===========================================================================
