#include "optimization_benchmarks.h"

void generateConditionedRandomProjection(arr& M, uint n, double condition){
  uint i,j;
  //let M be a ortho-normal matrix (=random rotation matrix)
  M.resize(n,n);
  rndUniform(M,-1.,1.,false);
  //orthogonalize
  for(i=0;i<n;i++){
    for(j=0;j<i;j++) M[i]()-=scalarProduct(M[i],M[j])*M[j];
    M[i]()/=norm(M[i]);
  }
  //we condition each column of M with powers of the condition
  for(i=0;i<n;i++) M[i]() *= pow(condition, double(i) / (2.*double(n - 1)));
}

SquaredCost::SquaredCost(uint _n, double condition){
  initRandom(_n, condition);
}

void SquaredCost::initRandom(uint _n, double condition){
  n=_n;
  generateConditionedRandomProjection(M, n, condition);
  //the metric is equal M^T*M
  C=~M*M;
  //arr U,d,V;    svd(U, d, V, C);    cout <<U <<d <<V <<M <<C <<endl;
}

double SquaredCost::fs(arr *grad,const arr& x){
  arr y;
  fv(y, grad, x);
  if(grad) *grad=2.*~y*(*grad);
  return sumOfSqr(y);
}

void SquaredCost::fv(arr& y, arr *J,const arr& x){
  CHECK(x.N==n,"");
  y = M*x;
  if(J) (*J)=M;
}

NonlinearlyWarpedSquaredCost::NonlinearlyWarpedSquaredCost(uint _n, double condition):sq(_n,condition){
  n=_n;
}

void NonlinearlyWarpedSquaredCost::initRandom(uint _n, double condition){
  n=_n;
  sq.initRandom(n,condition);
}
  
double NonlinearlyWarpedSquaredCost::fs(arr *grad,const arr& x){
  arr y;
  fv(y, grad, x);
  if(grad) *grad=2.*~y*(*grad);
  return sumOfSqr(y);
}

void NonlinearlyWarpedSquaredCost::fv(arr& y, arr *J,const arr& x){
  CHECK(x.N==n,"");
  arr xx=atan(x);
  y=sq.M*xx;
  if(J){
    arr gg(xx.N);
    for(uint i=0;i<gg.N;i++) gg(i) = 1./(1.+x(i)*x(i));
    *J = sq.M*diag(gg);
  }
}

VectorChainCost::VectorChainCost(uint _T,uint _n){
  T=_T; n=_n;
  A.resize(T+1,n,n);  a.resize(T+1,n);
  Wi.resize(T,n,n);  Wj.resize(T,n,n);    w.resize(T,n);
  for(uint t=0;t<=T;t++) generateConditionedRandomProjection(A[t](), n, 100.);
  for(uint t=0;t<T;t++){
    generateConditionedRandomProjection(Wi[t](), n, 100.);
    generateConditionedRandomProjection(Wj[t](), n, 100.);
  }
  rndUniform(a,-1.,1.,false);
  rndUniform(w,-1.,1.,false);
  nonlinear=false;
}

void VectorChainCost::fvi(arr& y, arr* J, uint i, const arr& x_i){
  if(!nonlinear){
    y = A[i]*x_i + a[i];
    if(J) *J = A[i];
  }else{
    arr xi=atan(x_i);
    y = A[i]*xi + a[i];
    if(J){
      arr gi(xi.N);
      for(uint k=0;k<gi.N;k++) gi(k) = 1./(1.+x_i(k)*x_i(k));
      *J = A[i]*diag(gi);
    }
  }
}

void VectorChainCost::fvij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j){
  if(!nonlinear){
    y=Wi[i]*x_i + Wj[i]*x_j + w[i];
    if(Ji) *Ji = Wi[i];
    if(Jj) *Jj = Wj[i];
  }else{
    arr xi=atan(x_i);
    arr xj=atan(x_j);
    y=Wi[i]*xi + Wj[i]*xj + w[i];
    if(Ji && Ji){
      arr gi(xi.N),gj(xi.N);
      for(uint k=0;k<gi.N;k++) gi(k) = 1./(1.+x_i(k)*x_i(k));
      for(uint k=0;k<gj.N;k++) gj(k) = 1./(1.+x_j(k)*x_j(k));
      *Ji = Wi[i]*diag(gi);
      *Jj = Wj[i]*diag(gj);
    }
  }
}
