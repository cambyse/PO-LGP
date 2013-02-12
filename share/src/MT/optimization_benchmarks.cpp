/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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


#include "optimization_benchmarks.h"
//#include "functions.h"

void generateConditionedRandomProjection(arr& M, uint n, double condition) {
  uint i,j;
  //let M be a ortho-normal matrix (=random rotation matrix)
  M.resize(n,n);
  rndUniform(M,-1.,1.,false);
  //orthogonalize
  for(i=0; i<n; i++) {
    for(j=0; j<i; j++) M[i]()-=scalarProduct(M[i],M[j])*M[j];
    M[i]()/=norm(M[i]);
  }
  //we condition each column of M with powers of the condition
  for(i=0; i<n; i++) M[i]() *= pow(condition, double(i) / (2.*double(n - 1)));
}

SquaredCost::SquaredCost(uint _n, double condition) {
  initRandom(_n, condition);
}

void SquaredCost::initRandom(uint _n, double condition) {
  n=_n;
  generateConditionedRandomProjection(M, n, condition);
  //the metric is equal M^T*M
  C=~M*M;
  //arr U,d,V;    svd(U, d, V, C);    cout <<U <<d <<V <<M <<C <<endl;
}

double SquaredCost::fs(arr& grad,const arr& x) {
  arr y;
  fv(y, grad, x);
  if(&grad) grad=2.*~y*grad;
  return sumOfSqr(y);
}

void SquaredCost::fv(arr& y, arr& J,const arr& x) {
  CHECK(x.N==n,"");
  y = M*x;
  if(&J) J=M;
}

NonlinearlyWarpedSquaredCost::NonlinearlyWarpedSquaredCost(uint _n, double condition):sq(_n,condition) {
  n=_n;
}

void NonlinearlyWarpedSquaredCost::initRandom(uint _n, double condition) {
  n=_n;
  sq.initRandom(n,condition);
}

double NonlinearlyWarpedSquaredCost::fs(arr& grad,const arr& x) {
  arr y;
  fv(y, grad, x);
  if(&grad) grad=2.*~y*grad;
  return sumOfSqr(y);
}

void NonlinearlyWarpedSquaredCost::fv(arr& y, arr& J,const arr& x) {
  CHECK(x.N==n,"");
  arr xx=atan(x);
  y=sq.M*xx;
  if(&J) {
    arr gg(xx.N);
    for(uint i=0; i<gg.N; i++) gg(i) = 1./(1.+x(i)*x(i));
    J = sq.M*diag(gg);
  }
}

VectorChainCost::VectorChainCost(uint _T,uint _n) {
  T=_T; n=_n;
  A.resize(T+1,n,n);  a.resize(T+1,n);
  Wi.resize(T,n,n);  Wj.resize(T,n,n);    w.resize(T,n);
  for(uint t=0; t<=T; t++) generateConditionedRandomProjection(A[t](), n, 100.);
  for(uint t=0; t<T; t++) {
    generateConditionedRandomProjection(Wi[t](), n, 100.);
    generateConditionedRandomProjection(Wj[t](), n, 100.);
  }
  rndUniform(a,-1.,1.,false);
  rndUniform(w,-1.,1.,false);
  nonlinear=false;
}

void VectorChainCost::fv_i(arr& y, arr* J, uint i, const arr& x_i) {
  if(!nonlinear) {
    y = A[i]*x_i + a[i];
    if(J) *J = A[i];
  } else {
    arr xi=atan(x_i);
    y = A[i]*xi + a[i];
    if(J) {
      arr gi(xi.N);
      for(uint k=0; k<gi.N; k++) gi(k) = 1./(1.+x_i(k)*x_i(k));
      *J = A[i]*diag(gi);
    }
  }
}

void VectorChainCost::fv_ij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j) {
  if(!nonlinear) {
    y=Wi[i]*x_i + Wj[i]*x_j + w[i];
    if(Ji) *Ji = Wi[i];
    if(Jj) *Jj = Wj[i];
  } else {
    arr xi=atan(x_i);
    arr xj=atan(x_j);
    y=Wi[i]*xi + Wj[i]*xj + w[i];
    if(Ji && Ji) {
      arr gi(xi.N),gj(xi.N);
      for(uint k=0; k<gi.N; k++) gi(k) = 1./(1.+x_i(k)*x_i(k));
      for(uint k=0; k<gj.N; k++) gj(k) = 1./(1.+x_j(k)*x_j(k));
      *Ji = Wi[i]*diag(gi);
      *Jj = Wj[i]*diag(gj);
    }
  }
}

SlalomProblem::SlalomProblem(uint _T, uint _K, double _margin, double _w, double _power) {
  T=_T;
  n=2;
  K=_K;
  margin = _margin;
  w = _w;
  power = _power;
}


double border(double *grad, double x, double power=8.) {
  if(x>0.) { if(grad) *grad=0.; return 0.; }
  double y = pow(x,power);
  if(grad) *grad = power*pow(x,power-1.);
  return y;
}

double tannenbaum(double *grad, double x, double power=8.) {
  double y=x*x;
  if(grad) *grad = power*pow(y-floor(y),power-1.) * (2.*x);
  y=floor(y) + pow(y-floor(y),power);
  return y;
}


void SlalomProblem::fv_i(arr& y, arr& J, uint i, const arr& x_i) {
  eval_cost++;
  CHECK(x_i.N==2,"");
  y.resize(1);  y(0)=0.;
  if(&J) { J.resize(1,2);  J.setZero(); }
  if(!(i%(T/K))) {
    uint obstacle=i/(T/K);
    if(obstacle&1) { //top obstacle
      double d=(x_i(0)-1.)/margin;
//       y(0) = tannenbaum((J?&(*J)(0,0):NULL), d, power);
      y(0) = border((&J?&J(0,0):NULL), d, power);
      if(&J) J(0,0) /= margin;
    } else {
      double d=-(x_i(0)+1.)/margin;
//       y(0) = tannenbaum((J?&J(0,0):NULL), d, power);
      y(0) = border((&J?&J(0,0):NULL), d, power);
      if(&J) J(0,0) /= -margin;
    }
  }
}

void SlalomProblem::fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j) {
  y.resize(1);
  double tau=.01;
  arr A=ARRAY(1., tau, 0., 1.);  A.reshape(2,2);
  arr M=w*diag(ARRAY(2./(tau*tau), 1./tau));  //penalize variance in position & in velocity (control)
  y=M*(x_j - A*x_i);
  if(&Ji) { Ji = -M*A; }
  if(&Jj) { Jj = M; }
}
