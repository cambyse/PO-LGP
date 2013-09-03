#include "constrained.h"

//==============================================================================
//
// UnconstrainedProblem
//

double UnconstrainedProblem::fs(arr& df, arr& Hf, const arr& x){
  arr g, Jg;
  double f = P.fc(df, Hf, g, (&df || &Hf?Jg:NoArr), x);
  CHECK(P.dim_g()==g.N,"this conversion requires phi.N to be m-dimensional");

  if(Hf.special) Hf = unpack(Hf);
  if(Jg.special) Jg = unpack(Jg);

  //in log barrier case, check feasibility
  if(muLB)     for(uint i=0;i<g.N;i++) if(g(i)>0.) return NAN; //CHECK(phi(i)<=0., "log barrier: constraints must be fulfiled!");

  if(muLB)     for(uint i=0;i<g.N;i++) f -= muLB * ::log(-g(i));  //log barrier
  if(mu)       for(uint i=0;i<g.N;i++) if(g(i)>0. || (lambda.N && lambda(i)>0.)) f += mu * MT::sqr(g(i));  //penalty
  if(lambda.N) for(uint i=0;i<g.N;i++) if(lambda(i)>0.) f += lambda(i) * g(i);  //augments

  if(&df){
    if(muLB)     for(uint i=0;i<g.N;i++) df -= (muLB/g(i))*Jg[i];  //log barrier
    if(mu)       for(uint i=0;i<g.N;i++) if(g(i)>0. || (lambda.N && lambda(i)>0.)) df += (mu*2.*g(i))*Jg[i];  //penalty
    if(lambda.N) for(uint i=0;i<g.N;i++) if(lambda(i)>0.) df += lambda(i)*Jg[i];  //augments
    if(!df.special) df.reshape(x.N);
  }

  if(&Hf){
    /// the 2.*Jg^T Jg terms are considered as in Gauss-Newton type; no real Hg used
    if(muLB)     for(uint i=0;i<g.N;i++) Hf += (muLB/MT::sqr(g(i)))*(Jg[i]^Jg[i]);  //log barrier
    if(mu)       for(uint i=0;i<g.N;i++) if(g(i)>0. || (lambda.N && lambda(i)>0.)) Hf += (mu*2.)*(Jg[i]^Jg[i]);  //penalty
    if(lambda.N) for(uint i=0;i<g.N;i++) if(lambda(i)>0.) Hf += 0.; //augments
    if(!Hf.special) Hf.reshape(x.N,x.N);
  }

  return f;
}

//void UnconstrainedProblem::fv(arr& y, arr& Jy, const arr& x){
//  arr phi, J;
//  f.fv(phi, (&Jy?J:NoArr), x);

//  //can't handle log barriers in GaussNewton case yet
//  if(muLB) NIY;
//  uint c=f.get_c(); //#constraints
//  uint m=phi.N-c;   //#costs
//  CHECK(m>=1 && c>=1,"");

//  //costs
//  y.resize(m + (mu?c:0) + (lambda.N?c:0)); y.setZero();
//  uint j=0;
//  for(uint i=0;i<m;i++,j++) y(j) = phi(i); //copy the plain GaussNewton costs
//  //if(muLB)     for(uint i=0;i<phi.N;i++,j++) f -= muLB * ::log(-phi(i));  //log barrier
//  if(mu)       for(uint i=m;i<phi.N;i++,j++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) y(j) = sqrt(mu) * phi(i);  //penalty
//  if(lambda.N) for(uint i=m;i<phi.N;i++,j++) if(lambda(i)>0.) y(j) = sqrt(lambda(i) * phi(i));  //augments

//  if(&Jy){
//    Jy.resize(y.N, x.N); Jy.setZero();
//    uint j=0;
//    for(uint i=0;i<m;i++,j++) Jy[j]() = J[i]; //copy the plain GaussNewton costs gradients
////    if(muLB)     for(uint i=m;i<phi.N;i++,j++) g -= (muLB/phi(i))*J[i];  //log barrier
//    if(mu)       for(uint i=m;i<phi.N;i++,j++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) Jy[j]() = sqrt(mu) * J[i];  //penalty
//    if(lambda.N) for(uint i=m;i<phi.N;i++,j++) if(lambda(i)>0.) Jy[j]() = .5*sqrt(lambda(i)/phi(i)) * J[i];  //augments
//  }
//}

void UnconstrainedProblem::augmentedLagrangian_LambdaUpdate(const arr& x){
  arr g;
  P.fc(NoArr, NoArr, g, NoArr, x);

  if(!lambda.N){ lambda.resize(g.N); lambda.setZero(); }

  for(uint i=0;i<g.N;i++) if(g(i)>0. || lambda(i)>0.) lambda(i) += mu * 2.*g(i);

  for(uint i=0;i<g.N;i++) if(lambda(i)<0.) lambda(i)=0.;

  cout <<"Update Lambda: phi=" <<g <<" lambda=" <<lambda <<endl;
}

//==============================================================================
//
// PhaseOneProblem
//


double PhaseOneProblem::fc(arr& df, arr& Hf, arr& meta_g, arr& meta_Jg, const arr& x){
  NIY;
  arr g, Jg;
  f.fc(NoArr, NoArr, g, (&meta_Jg?Jg:NoArr), x.sub(0,-2)); //the underlying problem only receives a x.N-1 dimensional x

  meta_g.resize(g.N+1);
  meta_g(0) = x.last();                                       //cost
  for(uint i=0;i<g.N;i++) meta_g(i) = g(i)-x.last();  //slack constraints
  meta_g.last() = -x.last();                                  //last constraint

  if(&meta_Jg){
    meta_Jg.resize(meta_g.N, x.N);  meta_Jg.setZero();
    meta_Jg(0,x.N-1) = 1.; //cost
    for(uint i=0;i<g.N;i++) for(uint j=0;j<x.N-1;j++) meta_Jg(i,j) = Jg(i,j);
    for(uint i=0;i<g.N;i++) meta_Jg(i,x.N-1) = -1.;
    meta_Jg(g.N, x.N-1) = -1.;
  }
}

