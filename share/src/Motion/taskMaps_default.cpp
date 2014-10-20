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

#include "taskMaps.h"

DefaultTaskMap::DefaultTaskMap(DefaultTaskMapType _type,
                               int iShape, const ors::Vector& _ivec,
                               int jShape, const ors::Vector& _jvec)
  :type(_type), i(iShape), j(jShape){
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
}

DefaultTaskMap::DefaultTaskMap(DefaultTaskMapType _type, const ors::KinematicWorld &G,
                               const char* iShapeName, const ors::Vector& _ivec,
                               const char* jShapeName, const ors::Vector& _jvec)
  :type(_type), i(-1), j(-1){
  ors::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  ors::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
}


void DefaultTaskMap::phi(arr& y, arr& J, const ors::KinematicWorld& G) {
  ors::Body *body_i = i<0?NULL: G.shapes(i)->body;
  ors::Body *body_j = j<0?NULL: G.shapes(j)->body;

  if(type==posTMT){
    ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsPos(y, J, body_i, &vec_i);
      y -= ARRAY(vec_j);
      return;
    }//else...
    ors::Vector pi = body_i->X * vec_i;
    ors::Vector pj = body_j->X * vec_j;
    y = ARRAY(body_j->X.rot / (pi-pj));
    if(&J) {
      arr Ji, Jj, JRj;
      G.kinematicsPos(NoArr, Ji, body_i, &vec_i);
      G.kinematicsPos(NoArr, Jj, body_j, &vec_j);
      G.jacobianR(JRj, body_j);
      J.resize(3, Jj.d1);
      for(uint k=0; k<Jj.d1; k++) {
        ors::Vector vi(Ji(0, k), Ji(1, k), Ji(2, k));
        ors::Vector vj(Jj(0, k), Jj(1, k), Jj(2, k));
        ors::Vector r (JRj(0, k), JRj(1, k), JRj(2, k));
        ors::Vector jk =  body_j->X.rot / (vi-vj);
        jk -= body_j->X.rot / (r ^ (pi-pj));
        J(0, k)=jk.x;
        J(1, k)=jk.y;
        J(2, k)=jk.z;
      }
    }
    return;
  }

  if(type==vecTMT){
    ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
//    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsVec(y, J, body_i, &vec_i);
      return;
    }//else...
    //relative
    MT_MSG("warning - don't have a correct Jacobian for this TMType yet");
    //      fi = G.bodies(body_i)->X; fi.appendTransformation(irel);
    //      fj = G.bodies(body_j)->X; fj.appendTransformation(jrel);
    //      f.setDifference(fi, fj);
    //      f.rot.getZ(c);
    //      y = ARRAY(c);
    NIY; //TODO: Jacobian?
    return;
  }

  if(type==vecAlignTMT) {
    CHECK(fabs(ivec.length()-1.)<1e-10,"vector references must be normalized");
    CHECK(fabs(jvec.length()-1.)<1e-10,"vector references must be normalized");
    ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    arr zi,Ji,zj,Jj;
    G.kinematicsVec(zi, Ji, body_i, &vec_i);
    if(body_j==NULL) {
      zj = ARRAY(vec_j);
      if(&J) { Jj.resizeAs(Ji); Jj.setZero(); }
    } else {
      G.kinematicsVec(zj, Jj, body_j, &vec_j);
    }
    y.resize(1);
    y(0) = scalarProduct(zi, zj);
    if(&J) {
      J = ~zj * Ji + ~zi * Jj;
      J.reshape(1, G.getJointStateDimension());
    }
    return;
  }

  if(type==quatTMT){
    if(body_j==NULL) {
      G.kinematicsQuat(y, J, body_i);
      return;
    }
    NIY;
    return;
  }

  HALT("no such TVT");
}

uint DefaultTaskMap::dim_phi(const ors::KinematicWorld& G) {
  switch(type) {
    case posTMT: return 3;
    case vecTMT: return 3;
    case vecAlignTMT: return 1;
    case quatTMT: return 4;
    default:  HALT("no such TMT");
  }
}

//===========================================================================

void TaskMap_qItself::phi(arr& q, arr& J, const ors::KinematicWorld& G) {
  G.getJointState(q);
  if(M.N){
    if(M.nd==1){
      q=M%q; if(&J) J.setDiag(M);
    }else{
      q=M*q; if(&J) J=M;
    }
  }else{
    if(&J) J.setId(q.N);
  }
}

uint TaskMap_qItself::dim_phi(const ors::KinematicWorld& G) {
  if(M.nd==2) return M.d0;
  return G.getJointStateDimension();
}

//===========================================================================

void TaskMap_qLimits::phi(arr& y, arr& J, const ors::KinematicWorld& G) {
  if(!limits.N) limits=G.getLimits();
  G.kinematicsLimitsCost(y, J, limits);
}
