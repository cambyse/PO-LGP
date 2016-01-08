#pragma once

#include "motion.h"

struct TaskMap_qItself:TaskMap {
  arr M;            ///< optionally, the task map is M*q or M%q (linear in q)
  bool moduloTwoPi; ///< if false, consider multiple turns of a joint as different q values (Default: true)

  TaskMap_qItself(uint singleQ, uint qN) : moduloTwoPi(true) { M=zeros(1,qN); M(0,singleQ)=1.; } ///< The singleQ parameter generates a matrix M that picks out a single q value
  TaskMap_qItself(const arr& _M=NoArr) : moduloTwoPi(true) { if(&_M) M=_M; }                     ///< Specifying NoArr returns q; specifying a vector M returns M%q; specifying a matrix M returns M*q
  TaskMap_qItself(const ors::KinematicWorld& G, ors::Joint* j)
    : moduloTwoPi(true)  {
    M = zeros(j->qDim(), G.getJointStateDimension() );
    M.setMatrixBlock(eye(j->qDim()), 0, j->qIndex);
  }
  TaskMap_qItself(const ors::KinematicWorld& G, const char* jointName)
    : moduloTwoPi(true)  {
    ors::Joint *j = G.getJointByName(jointName);
    M = zeros(j->qDim(), G.getJointStateDimension() );
    M.setMatrixBlock(eye(j->qDim()), 0, j->qIndex);
  }
  TaskMap_qItself(const ors::KinematicWorld& G, const char* jointName1, const char* jointName2)
    : moduloTwoPi(true)  {
    ors::Joint *j1 = G.getJointByName(jointName1);
    ors::Joint *j2 = G.getJointByName(jointName2);
    M = zeros(j1->qDim() + j2->qDim(), G.getJointStateDimension() );
    M.setMatrixBlock(eye(j1->qDim()), 0, j1->qIndex);
    M.setMatrixBlock(eye(j2->qDim()), j1->qDim(), j2->qIndex);
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1);
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t);
  virtual uint dim_phi(const ors::KinematicWorld& G);
  virtual uint dim_phi(const WorldL& G, int t);
private:
  uintA dimPhi;
};

//===========================================================================

struct TaskMap_qZeroVels:TaskMap {
  TaskMap_qZeroVels(){}

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1){NIY}
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t);
  virtual uint dim_phi(const ors::KinematicWorld& G){NIY}
  virtual uint dim_phi(const WorldL& G, int t);
private:
  uintA dimPhi;
};
