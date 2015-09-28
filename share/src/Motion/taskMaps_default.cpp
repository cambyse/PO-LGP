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

DefaultTaskMap::DefaultTaskMap(const Graph& specs, const ors::KinematicWorld& G)
  :type(noTMT), i(-1), j(-1){
  Node *it=specs["type"];
  if(it){
    MT::String Type=it->V<MT::String>();
         if(Type=="pos") type=posTMT;
    else if(Type=="vec") type=vecTMT;
    else if(Type=="quat") type=quatTMT;
    else if(Type=="posDiff") type=posDiffTMT;
    else if(Type=="vecDiff") type=vecDiffTMT;
    else if(Type=="quatDiff") type=quatDiffTMT;
    else if(Type=="vecAlign") type=vecAlignTMT;
    else if(Type=="gazeAt") type=gazeAtTMT;
    else HALT("unknown type " <<Type);
  }else HALT("no type given");
  if((it=specs["ref1"])){ auto name=it->V<MT::String>(); auto *s=G.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); i=s->index; }
  if((it=specs["ref2"])){ auto name=it->V<MT::String>(); auto *s=G.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); j=s->index; }
  if((it=specs["vec1"])) ivec = ors::Vector(it->V<arr>());  else ivec.setZero();
  if((it=specs["vec2"])) jvec = ors::Vector(it->V<arr>());  else jvec.setZero();
}


void DefaultTaskMap::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  if(t>=0 && referenceIds.N){
    if(referenceIds.nd==1){  i=referenceIds(t); j=-1; }
    if(referenceIds.nd==2){  i=referenceIds(t,0); j=referenceIds(t,1); }
  }

  ors::Body *body_i = i<0?NULL: G.shapes(i)->body;
  ors::Body *body_j = j<0?NULL: G.shapes(j)->body;

  if(type==posTMT){
    ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsPos(y, J, body_i, vec_i);
      y -= ARRAY(vec_j);
      return;
    }//else...
    ors::Vector pi = body_i->X * vec_i;
    ors::Vector pj = body_j->X * vec_j;
    y = ARRAY(body_j->X.rot / (pi-pj));
    if(&J) {
      arr Ji, Jj, JRj;
      G.kinematicsPos(NoArr, Ji, body_i, vec_i);
      G.kinematicsPos(NoArr, Jj, body_j, vec_j);
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

  if(type==posDiffTMT){
    ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    G.kinematicsPos(y, J, body_i, vec_i);
    if(!body_j){ //relative to world
      y -= ARRAY(vec_j);
    }else{
      arr y2, J2;
      G.kinematicsPos(y2, (&J?J2:NoArr), body_j, vec_j);
      y -= y2;
      if(&J) J -= J2;
    }
    return;
  }

  if(type==vecTMT){
    ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
//    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsVec(y, J, body_i, vec_i);
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

  if(type==vecDiffTMT){
    ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    G.kinematicsVec(y, J, body_i, vec_i);
    if(!body_j){ //relative to world
      y -= ARRAY(vec_j);
    }else{
      arr y2, J2;
      G.kinematicsVec(y2, J2, body_j, vec_j);
      y -= y2;
      J -= J2;
    }
    return;
  }

  if(type==vecAlignTMT) {
    CHECK(fabs(ivec.length()-1.)<1e-10,"vector references must be normalized");
    CHECK(fabs(jvec.length()-1.)<1e-10,"vector references must be normalized");
    ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel.rot*ivec;
    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel.rot*jvec;
    arr zi,Ji,zj,Jj;
    G.kinematicsVec(zi, Ji, body_i, vec_i);
    if(body_j==NULL) {
      zj = ARRAY(vec_j);
      if(&J) { Jj.resizeAs(Ji); Jj.setZero(); }
    } else {
      G.kinematicsVec(zj, Jj, body_j, vec_j);
    }
    y.resize(1);
    y(0) = scalarProduct(zi, zj);
    if(&J) {
      J = ~zj * Ji + ~zi * Jj;
      J.reshape(1, G.getJointStateDimension());
    }
    return;
  }

  if(type==gazeAtTMT){
    CHECK(i>=0, "ref1 is not set!");
    if(ivec.length()<1e-10) ivec.set(0.,0.,-1.);
    ors::Vector vec_i = G.shapes(i)->rel.rot*ivec;
    ors::Vector vec_xi = G.shapes(i)->rel.rot*Vector_x;
    ors::Vector vec_yi = G.shapes(i)->rel.rot*Vector_y;
    ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
    arr pi,Jpi, xi,Jxi, yi,Jyi, pj,Jpj;
    G.kinematicsPos(pi, Jpi, body_i, vec_i);
    G.kinematicsVec(xi, Jxi, body_i, vec_xi);
    G.kinematicsVec(yi, Jyi, body_i, vec_yi);
    if(body_j==NULL) {
      pj = ARRAY(vec_j);
      if(&J) { Jpj.resizeAs(Jpi); Jpj.setZero(); }
    } else {
      G.kinematicsPos(pj, Jpj, body_j, vec_j);
    }
    y.resize(2);
    y(0) = scalarProduct(xi, (pj-pi));
    y(1) = scalarProduct(yi, (pj-pi));
    if(&J) {
      J = cat( ~xi * (Jpj-Jpi) + ~(pj-pi) * Jxi,
               ~yi * (Jpj-Jpi) + ~(pj-pi) * Jyi );
      J.reshape(2, G.getJointStateDimension());
    }
    return;
  }

  if(type==quatTMT){
    if(body_j==NULL) { //simple, no j reference
      G.kinematicsQuat(y, J, body_i);
      return;
    }//else...
    NIY;
  }

  if(type==quatDiffTMT){
    ors::Quaternion q_i; if(i>=0) q_i=G.shapes(i)->rel.rot; else q_i.setZero();
    ors::Quaternion q_j; if(j>=0) q_j=G.shapes(j)->rel.rot; else q_j.setZero();
    G.kinematicsQuat(y, J, body_i);
    if(!body_j){ //relative to world
//      arr y2 = ARRAY(q_j);
//      if(scalarProduct(y,y2)>=0.){
//        y -= y2;
//      }else{
//        y += y2;
//      }
    }else{
      arr y2, J2;
      G.kinematicsQuat(y2, J2, body_j);
      if(scalarProduct(y,y2)>=0.){
        y -= y2;
        J -= J2;
      }else{
        y += y2;
        J += J2;
      }
    }
    return;
  }

  HALT("no such TVT");
}

uint DefaultTaskMap::dim_phi(const ors::KinematicWorld& G) {
  switch(type) {
    case posTMT: return 3;
    case vecTMT: return 3;
    case quatTMT: return 4;
    case posDiffTMT: return 3;
    case vecDiffTMT: return 3;
    case quatDiffTMT: return 4;
    case vecAlignTMT: return 1;
    case gazeAtTMT: return 2;
    default:  HALT("no such TMT");
  }
}

//===========================================================================

void TaskMap_qItself::phi(arr& q, arr& J, const ors::KinematicWorld& G, int t) {
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

void TaskMap_qLimits::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  if(!limits.N) limits=G.getLimits();
  G.kinematicsLimitsCost(y, J, limits);
}

//===========================================================================
