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
                   const char* jShapeName, const ors::Vector& _jvec) {
  ors::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  ors::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
  if(&_ivec) ivec=_ivec; else ivec.setZero();
  if(&_jvec) jvec=_jvec; else jvec.setZero();
  type=ineqTT;
  order = 1;
}

void VelAlignConstraint::phi(arr &y, arr &J, const ors::KinematicWorld &G) {
  arr y1,J1;
  G.kinematicsPos(y1,J1,G.shapes(i)->body,&ivec);

  arr y2,J2;
  G.kinematicsVec(y2,J2,G.shapes(j)->body,&jvec);
  innerProduct(y,~y1,y2);
  J = ~y2*J1 + ~y1*J2;
//  cout << "y1: " << y1 << endl;
//  cout << "y2: " << y2 << endl;
}
