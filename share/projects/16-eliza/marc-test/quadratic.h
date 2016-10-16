#pragma once

#include <Core/array.h>

#include <RL/RL.h>

struct QuadraticQ : mlr::QFunction{
  uint ds,da,n;
  arr beta, Beta;

  QuadraticQ(uint ds, uint da):ds(ds), da(da){
    n = ds+da;
    beta = zeros(symIndex(n-1,n-1,n)+1);
    Beta = zeros(n,n);
  }

  ~QuadraticQ() {}

  double operator()(const arr& inputFeatures, const arr& action){
#if 1
    arr x = cat(inputFeatures, action);
    return scalarProduct(x,Beta*x);
#else
    arr phi = getFeatures(inputFeatures, action);
    return scalarProduct(phi, beta);
#endif
  }

  arr getMaxAction(const arr& inputFeatures){
    arr theta = getPolicy();
    return theta * inputFeatures;
  }

  void setBeta(const arr& _beta){
    if(&beta!=&beta) beta = _beta;
    for(uint i=0;i<n;i++) for(uint j=i;j<n;j++){
      if(i==j) Beta(i,j) = beta(symIndex(i,j,n));
      else Beta(i,j) = Beta(j,i) = .5* beta(symIndex(i,j,n));
    }
  }

  arr getFeatures(const arr& inputFeatures, const arr& action){
    arr phi(beta.N);
    arr x = cat(inputFeatures, action); //state-action vector
    arr xx = x ^ x;
    for(uint i=0;i<x.N;i++) for(uint j=i;j<x.N;j++) phi(symIndex(i,j,n)) = xx(i,j);
    return phi;
  }

  double train(const arr& S, const arr&A, const arr& y){
    CHECK(S.d0==A.d0 && S.d0==y.N,"");

    arr phi(S.d0, beta.N);
    for(uint m=0;m<S.d0;m++) phi[m] = getFeatures(S[m], A[m]);

    beta = inverse_SymPosDef(~phi * phi)* ~phi * y;

    double mse = sumOfSqr(y-phi*beta)/double(y.N);
//    cout <<"y-Var = " <<sumOfSqr(y)/double(y.N) <<" MSE = " <<mse <<endl;
    setBeta(beta);
//    cout <<beta <<"\n\n" <<Beta <<endl;
    return mse;
  }


  arr getBeta_sa(){
    arr B(ds,da);
    for(uint i=0;i<ds;i++) for(uint j=0;j<da;j++) B(i,j) = Beta(i, ds+j);
    return B;
  }

  arr getBeta_aa(){
    arr B(da,da);
    for(uint i=0;i<da;i++) for(uint j=0;j<da;j++) B(i,j) = Beta(ds+i, ds+j);
    return B;
  }

  arr getPolicy(){
    arr Baa = getBeta_aa();
    return  -inverse(~Baa+1e-10*eye(da)) * ~getBeta_sa();
  }

  arr gradA(const arr& s, const arr& a){
    arr g(da);
    arr x = cat(s, a);
    for(uint k=0;k<da;k++){
      double s = 0.;
      for(uint j=0;j<x.N;j++) s += x(j)*Beta(j, ds+k);
      g(k) = 2.*s;
    }
    return g;
  }

  arr hessianA(){
    arr H(da,da);
    for(uint k=0;k<da;k++) for(uint l=0;l<=k;l++){
      H(k,l) = H(l,k) = 2. * Beta(1+ds+k, 1+ds+l);
    }
    return H;
  }
};
