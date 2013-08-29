#include "constrained.h"

//==============================================================================
//
// UnconstrainedProblem
//

double UnconstrainedProblem::fs(arr& g, arr& H, const arr& x){
  arr phi, J;
  f.fv(phi, (&g?J:NoArr), x);
  CHECK(f.get_c()==phi.N-1,"this conversion requires phi.N to be 1+#constraints");


  //in log barrier case, check feasibility
  if(muLB)     for(uint i=1;i<phi.N;i++) if(phi(i)>0.) return NAN; //CHECK(phi(i)<=0., "log barrier: constraints must be fulfiled!");

  double f = phi(0); //costs
  if(muLB)     for(uint i=1;i<phi.N;i++) f -= muLB * ::log(-phi(i));  //log barrier
  if(mu)       for(uint i=1;i<phi.N;i++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) f += mu * MT::sqr(phi(i));  //penalty
  if(lambda.N) for(uint i=1;i<phi.N;i++) if(lambda(i)>0.) f += lambda(i) * phi(i);  //augments

  if(&g){
    g = J[0]; //costs
    if(muLB)     for(uint i=1;i<phi.N;i++) g -= (muLB/phi(i))*J[i];  //log barrier
    if(mu)       for(uint i=1;i<phi.N;i++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) g += (mu*2.*phi(i))*J[i];  //penalty
    if(lambda.N) for(uint i=1;i<phi.N;i++) if(lambda(i)>0.) g += lambda(i)*J[i];  //augments
    g.reshape(x.N);
  }

  if(&H){
    ///TODO: Here we assume the hessian of phi(0) and all phi(i) ZERO!!! Only the J^T J terms are considered (as in Gauss-Newton type)
    H.resize(x.N,x.N);
    H.setZero();
    if(muLB)     for(uint i=1;i<phi.N;i++) H += (muLB/MT::sqr(phi(i)))*(J[i]^J[i]);  //log barrier
    if(mu)       for(uint i=1;i<phi.N;i++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) H += (mu*2.)*(J[i]^J[i]);  //penalty
    if(lambda.N) for(uint i=1;i<phi.N;i++) if(lambda(i)>0.) H += 0.; //augments
    H.reshape(x.N,x.N);
  }

  return f;
}

void UnconstrainedProblem::fv(arr& y, arr& Jy, const arr& x){
  arr phi, J;
  f.fv(phi, (&Jy?J:NoArr), x);

  //can't handle log barriers in GaussNewton case yet
  if(muLB) NIY;
  uint c=f.get_c(); //#constraints
  uint m=phi.N-c;   //#costs
  CHECK(m>=1 && c>=1,"");

  //costs
  y.resize(m + (mu?c:0) + (lambda.N?c:0)); y.setZero();
  uint j=0;
  for(uint i=0;i<m;i++,j++) y(j) = phi(i); //copy the plain GaussNewton costs
  //if(muLB)     for(uint i=1;i<phi.N;i++,j++) f -= muLB * ::log(-phi(i));  //log barrier
  if(mu)       for(uint i=m;i<phi.N;i++,j++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) y(j) = sqrt(mu) * phi(i);  //penalty
  if(lambda.N) for(uint i=m;i<phi.N;i++,j++) if(lambda(i)>0.) y(j) = sqrt(lambda(i) * phi(i));  //augments

  if(&Jy){
    Jy.resize(y.N, x.N); Jy.setZero();
    uint j=0;
    for(uint i=0;i<m;i++,j++) Jy[j]() = J[i]; //copy the plain GaussNewton costs gradients
//    if(muLB)     for(uint i=m;i<phi.N;i++,j++) g -= (muLB/phi(i))*J[i];  //log barrier
    if(mu)       for(uint i=m;i<phi.N;i++,j++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) Jy[j]() = sqrt(mu) * J[i];  //penalty
    if(lambda.N) for(uint i=m;i<phi.N;i++,j++) if(lambda(i)>0.) Jy[j]() = .5*sqrt(lambda(i)/phi(i)) * J[i];  //augments
  }
}

void UnconstrainedProblem::augmentedLagrangian_LambdaUpdate(const arr& x){
  arr phi;
  f.fv(phi, NoArr, x);

  if(!lambda.N){ lambda.resize(phi.N); lambda.setZero(); }

  for(uint i=1;i<phi.N;i++) if(phi(i)>0. || lambda(i)>0.) lambda(i) += mu * 2.*phi(i);

  for(uint i=1;i<phi.N;i++) if(lambda(i)<0.) lambda(i)=0.;

  cout <<"Update Lambda: phi=" <<phi <<" lambda=" <<lambda <<endl;
}

//==============================================================================
//
// PhaseOneProblem
//


void PhaseOneProblem::fv(arr& metaPhi, arr& metaJ, const arr& x){
  arr phi, J;
  f.fv(phi, (&metaJ?J:NoArr), x.sub(0,-2)); //the underlying problem only receives a x.N-1 dimensional x

  metaPhi.resize(phi.N+1);
  metaPhi(0) = x.last();                                     //cost
  for(uint i=1;i<phi.N;i++) metaPhi(i) = phi(i)-x.last();    //slack constraints
  metaPhi.last() = -x.last();                                //last constraint

  if(&metaJ){
    metaJ.resize(metaPhi.N, x.N);  metaJ.setZero();
    metaJ(0,x.N-1) = 1.; //cost
    for(uint i=1;i<phi.N;i++) for(uint j=0;j<x.N-1;j++) metaJ(i,j) = J(i,j);
    for(uint i=1;i<phi.N;i++) metaJ(i,x.N-1) = -1.;
    metaJ(phi.N, x.N-1) = -1.;
  }
}

