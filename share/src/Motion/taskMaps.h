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

#include "taskMap.h"

#include "taskMap_qItself.h"
#include "taskMap_GJK.h"
#include "taskMap_transition.h"
#include "taskMap_default.h"
#include "taskMap_qLimits.h"


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
  
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G);
};

//===========================================================================

struct CollisionConstraint:TaskMap {
  double margin;
  CollisionConstraint(double _margin=.1):margin(_margin){}
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return STRING("CollisionConstraint"); }
};


//===========================================================================

struct ProxyConstraint:TaskMap {
  TaskMap_Proxy proxyCosts;
  ProxyConstraint(PTMtype _type,
                  uintA _shapes,
                  double _margin=.02,
                  bool _useCenterDist=false,
                  bool _useDistNotCost=false);
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct LimitsConstraint:TaskMap {
  double margin;
  arr limits;
  LimitsConstraint(double _margin=.05):margin(_margin){}
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
  virtual mlr::String shortTag(const ors::KinematicWorld& G){ return STRING("LimitsConstraint"); }
};

//===========================================================================

struct PairCollisionConstraint:TaskMap {
  int i,j;       ///< which shapes does it refer to?
  double margin;
  intA referenceIds; ///< the shapes it refers to DEPENDENT on time
  PairCollisionConstraint(double _margin)
    : i(-1), j(-1), margin(_margin){
  }
  PairCollisionConstraint(const ors::KinematicWorld& G, const char* iShapeName, const char* jShapeName, double _margin=.02)
    : i(G.getShapeByName(iShapeName)->index),
      j(G.getShapeByName(jShapeName)->index),
      margin(_margin) {
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PlaneConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  arr planeParams;  ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  PlaneConstraint(const ors::KinematicWorld& G, const char* iShapeName, const arr& _planeParams)
    : i(G.getShapeByName(iShapeName)->index), planeParams(_planeParams){}

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

//this is NOT a constraint -- it turns a constraint into stickiness
struct ConstraintStickiness:TaskMap {
  TaskMap& map;
  ConstraintStickiness(TaskMap& _map)
    : map(_map) {
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PointEqualityConstraint:TaskMap {
  int i, j;               ///< which shapes does it refer to?
  ors::Vector ivec, jvec; ///< additional position or vector

  PointEqualityConstraint(const ors::KinematicWorld &G,
                          const char* iShapeName=NULL, const ors::Vector& _ivec=NoVector,
                          const char* jShapeName=NULL, const ors::Vector& _jvec=NoVector){
    TaskMap_Default dummy(posTMT, G, iShapeName, _ivec, jShapeName, _jvec); //is deleted in a sec..
    i=dummy.i;
    j=dummy.j;
    ivec=dummy.ivec;
    jvec=dummy.jvec;
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 3; }
};

//===========================================================================

struct ContactEqualityConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  double margin;
  ContactEqualityConstraint(const ors::KinematicWorld& G, const char* iShapeName, const char* jShapeName,double _margin)
    : i(G.getShapeByName(iShapeName)->index),
      j(G.getShapeByName(jShapeName)->index),
      margin(_margin) {
  }
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const ors::KinematicWorld& G){
    return 1;
  }
};

//===========================================================================

struct VelAlignConstraint:TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  ors::Vector ivec, jvec; ///< additional position or vector
  double target;

  double margin;
  VelAlignConstraint(const ors::KinematicWorld& G,
                     const char* iShapeName=NULL, const ors::Vector& _ivec=NoVector,
                     const char* jShapeName=NULL, const ors::Vector& _jvec=NoVector, double _target = 0.);

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=1) { } ;
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct qItselfConstraint:TaskMap {
  arr M;

  qItselfConstraint(uint singleQ, uint qN){ M=zeros(1,qN); M(0,singleQ)=1.; }
  qItselfConstraint(const arr& _M=NoArr){ if(&_M) M=_M; }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=1);
  virtual uint dim_phi(const ors::KinematicWorld& G){
    if(M.nd==2) return M.d0;
    return G.getJointStateDimension();
  }
};

//===========================================================================
