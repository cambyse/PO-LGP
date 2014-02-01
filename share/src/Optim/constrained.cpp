#include "constrained.h"

//==============================================================================
//
// UnconstrainedProblem
//

double UnconstrainedProblem::fs(arr& df, arr& Hf, const arr& x){
  arr g, Jg;
  double f = P.fc(df, Hf, g, (&df||&Hf ? Jg : NoArr), x);
  CHECK(P.dim_g()==g.N,"this conversion requires phi.N to be m-dimensional");

//  if(&Hf && Hf.special) Hf = unpack(Hf);
//  if(Jg.special) Jg = unpack(Jg);

  //in log barrier case, check feasibility
  if(muLB)     for(uint i=0;i<g.N;i++) if(g(i)>0.) return NAN; //CHECK(phi(i)<=0., "log barrier: constraints must be fulfiled!");

  if(muLB)     for(uint i=0;i<g.N;i++) f -= muLB * ::log(-g(i));  //log barrier
  if(mu)       for(uint i=0;i<g.N;i++) if(g(i)>0. || (lambda.N && lambda(i)>0.)) f += mu * MT::sqr(g(i));  //penalty
  if(lambda.N) for(uint i=0;i<g.N;i++) if(lambda(i)>0.) f += lambda(i) * g(i);  //augments

  if(&df){
    arr coeff(Jg.d0); coeff.setZero();
    if(muLB)     for(uint i=0;i<g.N;i++) coeff(i) -= (muLB/g(i));  //log barrier
    if(mu)       for(uint i=0;i<g.N;i++) if(g(i)>0. || (lambda.N && lambda(i)>0.)) coeff(i) += (mu*2.*g(i));  //penalty
    if(lambda.N) for(uint i=0;i<g.N;i++) if(lambda(i)>0.) coeff(i) += lambda(i);  //augments
    df += comp_At_x(Jg, coeff);
    df.reshape(x.N);
  }

  if(&Hf){
    /// the 2.*Jg^T Jg terms are considered as in Gauss-Newton type; no real Hg used
    arr coeff(Jg.d0); coeff.setZero();
    if(muLB)     for(uint i=0;i<g.N;i++) coeff(i) += (muLB/MT::sqr(g(i)));  //log barrier
    if(mu)       for(uint i=0;i<g.N;i++) if(g(i)>0. || (lambda.N && lambda(i)>0.)) coeff(i) += (mu*2.);  //penalty
    if(lambda.N) for(uint i=0;i<g.N;i++) if(lambda(i)>0.) coeff(i) += 0.; //augments
    for(uint i=0;i<g.N;i++) Jg[i]() *= sqrt(coeff(i));
    Hf += comp_At_A(Jg); //Gauss-Newton type!
    if(!Hf.special) Hf.reshape(x.N,x.N);
  }

  return f;
}

void UnconstrainedProblem::augmentedLagrangian_LambdaUpdate(const arr& x, double lambdaStepsize){
  arr g;
  P.fc(NoArr, NoArr, g, NoArr, x);

  if(!lambda.N){ lambda.resize(g.N); lambda.setZero(); }

  for(uint i=0;i<g.N;i++) if(g(i)>0. || lambda(i)>0.) lambda(i) += lambdaStepsize * mu * 2.*g(i);

  for(uint i=0;i<g.N;i++) if(lambda(i)<0.) lambda(i)=0.;

//  cout <<"Update Lambda: g=" <<g <<" lambda=" <<lambda <<endl;
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


//==============================================================================
//
// Solvers
//

const char* MethodName[]={ "NoMethod", "SquaredPenalty", "AugmentedLagrangian", "LogBarrier" };

void optConstrained(arr& x, ConstrainedProblem& P, OptOptions opt){
  UnconstrainedProblem UCP(P);

  //switch on penalty terms
  switch(opt.constrainedMethod){
    case squaredPenalty: UCP.mu=1.;  break;
    case augmentedLag:   UCP.mu=1.;  break;
    case logBarrier:     UCP.muLB=1.;  break;
  }

  if(opt.verbose>1) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<endl;

  for(uint k=0;;k++){
    if(opt.verbose>1) cout <<"***** optConstrained: iteration=" <<k
                             <<" mu=" <<UCP.mu <<" lambda=" <<UCP.lambda <<" muLB=" <<UCP.muLB <<endl;
    arr x_old = x;
    optNewton(x, UCP, opt);

    //upate unconstraint problem parameters
    switch(opt.constrainedMethod){
      case squaredPenalty: UCP.mu *= 10;  break;
      case augmentedLag:   UCP.augmentedLagrangian_LambdaUpdate(x);  break;
      case logBarrier:     UCP.muLB /= 2;  break;
    }

    //stopping criteron
    if(k>10 && absMax(x_old-x)<opt.stopTolerance){ cout << " --- optConstrained StoppingCriterion Delta<" <<opt.stopTolerance <<endl;  break; }
  }
}

