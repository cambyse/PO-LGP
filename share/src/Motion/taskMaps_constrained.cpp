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

//===========================================================================

void CollisionConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  G.kinematicsProxyCost(y, J, margin, false);
  y -= .5;
}

//===========================================================================

void LimitsConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  if(!limits.N) limits = G.getLimits();
  G.kinematicsLimitsCost(y, J, limits, margin);
  y -= .5;
}

//===========================================================================

void PairCollisionConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  y.resize(1) = -1.;
  if(&J) J.resize(1,G.q.N).setZero();
  for(ors::Proxy *p: G.proxies){
    if((p->a==i && p->b==j) || (p->a==j && p->b==i)){
      G.kinematicsProxyConstraint(y, J, p, margin, false);
      break;
    }
  }
}

//===========================================================================

void PlaneConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  ors::Body *body_i = G.shapes(i)->body;
  ors::Vector vec_i = G.shapes(i)->rel.pos;

  arr y_eff, J_eff;
  G.kinematicsPos(y_eff, (&J?J_eff:NoArr), body_i, &vec_i);

  y_eff.append(1.); //homogeneous coordinates
  if(&J) J_eff.append(zeros(1,J_eff.d1));

  y.resize(1);
  y(0) = scalarProduct(y_eff, planeParams);
  if(&J) J = ~planeParams * J_eff;
}

//===========================================================================

void ConstraintStickiness::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  map.phi(y, J, G);
  for(uint j=0;j<y.N;j++) y(j) = -y(j);
  if(&J) for(uint j=0;j<J.d0;j++) J[j]() *= -1.;
}

//===========================================================================

void PointEqualityConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
  ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;
  ors::Body *body_i = i<0?NULL: G.shapes(i)->body;
  ors::Body *body_j = j<0?NULL: G.shapes(j)->body;
  ors::Vector pi = body_i ? body_i->X * vec_i : vec_i;
  ors::Vector pj = body_j ? body_j->X * vec_j : vec_j;
  y = ARRAY(pi-pj);
  if(&J) {
    arr Ji, Jj;
    G.kinematicsPos(NoArr, Ji, body_i, &vec_i);
    G.kinematicsPos(NoArr, Jj, body_j, &vec_j);
    J = Ji - Jj;
  }
}

//===========================================================================

void ContactEqualityConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  y.resize(1) = 0.;
  if(&J) J.resize(1,G.q.N).setZero();
  for(ors::Proxy *p: G.proxies){
    if((p->a==i && p->b==j) || (p->a==j && p->b==i)){
      G.kinematicsProxyConstraint(y, J, p, margin, false);
      cout << y << endl;
      break;
    }
  }
}

VelAlignConstraint::VelAlignConstraint(const ors::KinematicWorld& G,
                   const char* iShapeName, const ors::Vector& _ivec,
                   const char* jShapeName, const ors::Vector& _jvec, double _target) {
  ors::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  ors::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
  type=ineqTT;
  order = 1;
  target = _target;
}

void VelAlignConstraint::phi(arr& y, arr& J, const WorldL& G, double tau) {
  uint k=order;

  // compute body j orientation
  arr y_j,J_j,J_bar_j;
  G(G.N-1)->kinematicsVec(y_j,(&J?J_bar_j:NoArr),G(G.N-1)->shapes(j)->body,&jvec);

  if(&J){
    J_j = zeros(G.N, y_j.N, J_bar_j.d1);
    J_j[G.N-1]() = J_bar_j;
    arr tmp(J_j);
    tensorPermutation(J_j, tmp, TUP(1,0,2));
    J_j.reshape(y_j.N, G.N*J_bar_j.d1);
  }

  // compute body i velocity
  arrA y_bar, J_bar;
  y_bar.resize(k+1);
  J_bar.resize(k+1);

  for(uint c=0;c<=k;c++) {
    G(G.N-1-c)->kinematicsPos(y_bar(c),(&J?J_bar(c):NoArr),G(G.N-1-c)->shapes(i)->body,&ivec);
  }

  arr dy_i, dJ_i;
  dy_i = (y_bar(0)-y_bar(1));

  if (&J) {
    dJ_i = zeros(G.N, dy_i.N, J_bar(0).d1);
    dJ_i[G.N-1-1]() = -J_bar(1);
    dJ_i[G.N-1-0]() = J_bar(0);
    arr tmp(dJ_i);
    tensorPermutation(dJ_i, tmp, TUP(1,0,2));
    dJ_i.reshape(dy_i.N, G.N*J_bar(0).d1);
  }

  // normalize dy_i
  if (length(dy_i) != 0) {
    if (&J) {
      double tmp = (~dy_i*dy_i).scalar();
      dJ_i = ( eye(dJ_i.d0) - (dy_i*~dy_i)/(tmp) )*dJ_i/(length(dy_i));
    }
    dy_i = dy_i/(length(dy_i));
  }

  innerProduct(y,~dy_i,y_j);

  if (&J) {
    J = ~dy_i*J_j + ~y_j*dJ_i;
    J = -J;
  }
  y = -y+target;

}
