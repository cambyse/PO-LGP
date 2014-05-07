#include "constrained.h"
#include "opt-newton.h"

//==============================================================================
//
// UnconstrainedProblem
//

double UnconstrainedProblem::fs(arr& dF, arr& HF, const arr& _x){
  //-- evaluate constrained problem and buffer
  if(_x!=x){
    x=_x;
    f_x = P.fc(df_x, Hf_x, g_x, Jg_x, x);
    CHECK(P.dim_g()==g_x.N,"this conversion requires phi.N to be m-dimensional");
  }else{ //we evaluated this before - use buffered values; the meta F is still recomputed as (dual) parameters might have changed
    if(&dF) CHECK(df_x.N && Jg_x.N,"");
    if(&HF) CHECK(Hf_x.N && Jg_x.N,"");
  }

  //  cout <<"g= " <<g_x <<" lambda= " <<lambda <<endl;

  //-- construct unconstrained problem
  double F=f_x;
  if(muLB)     for(uint i=0;i<g_x.N;i++){ if(g_x(i)>0.) return NAN;  F -= muLB * ::log(-g_x(i)); } //log barrier, check feasibility
  if(mu)       for(uint i=0;i<g_x.N;i++) if(g_x(i)>0. || (lambda.N && lambda(i)>0.)) F += mu * MT::sqr(g_x(i));  //penalty
  if(lambda.N) for(uint i=0;i<g_x.N;i++) if(lambda(i)>0.) F += lambda(i) * g_x(i);  //augments

  if(&dF){
    dF=df_x;
    arr coeff(Jg_x.d0); coeff.setZero();
    if(muLB)     for(uint i=0;i<g_x.N;i++) coeff(i) -= (muLB/g_x(i));  //log barrier
    if(mu)       for(uint i=0;i<g_x.N;i++) if(g_x(i)>0. || (lambda.N && lambda(i)>0.)) coeff(i) += (mu*2.*g_x(i));  //penalty
    if(lambda.N) for(uint i=0;i<g_x.N;i++) if(lambda(i)>0.) coeff(i) += lambda(i);  //augments
    dF += comp_At_x(Jg_x, coeff);
    dF.reshape(x.N);
  }

  if(&HF){
    HF=Hf_x;
    /// the 2.*Jg_x^T Jg_x terms are considered as in Gauss-Newton type; no real Hg used
    arr coeff(Jg_x.d0); coeff.setZero();
    if(muLB)     for(uint i=0;i<g_x.N;i++) coeff(i) += (muLB/MT::sqr(g_x(i)));  //log barrier
    if(mu)       for(uint i=0;i<g_x.N;i++) if(g_x(i)>0. || (lambda.N && lambda(i)>0.)) coeff(i) += (mu*2.);  //penalty
    if(lambda.N) for(uint i=0;i<g_x.N;i++) if(lambda(i)>0.) coeff(i) += 0.; //augments
    arr Jg_dg = Jg_x;
    for(uint i=0;i<g_x.N;i++) Jg_dg[i]() *= sqrt(coeff(i));
    HF += comp_At_A(Jg_dg); //Gauss-Newton type!
    if(!HF.special) HF.reshape(x.N,x.N);
  }

  return F;
}

void UnconstrainedProblem::aulaUpdate(double lambdaStepsize, arr& x_reeval){
  if(&x_reeval){
    x=x_reeval;
    f_x = P.fc(df_x, Hf_x, g_x, Jg_x, x);
  }

  if(!lambda.N){ lambda.resize(g_x.N); lambda.setZero(); }

  lambda += (lambdaStepsize * 2.*mu)*g_x;
  for(uint i=0;i<lambda.N;i++) if(lambda(i)<0.) lambda(i)=0.;

  cout <<"Update Lambda: g=" <<g_x <<" lambda=" <<lambda <<endl;
}

void UnconstrainedProblem::anyTimeAulaUpdate(double lambdaStepsize, double muInc, double *F_x, arr& dF_x, arr& HF_x){
  if(!lambda.N){ lambda.resize(g_x.N); lambda.setZero(); }

  arr lambdaOld = lambda;

  //collect gradients of active constraints
  arr A;
  if(Jg_x.special==arr::RowShiftedPackedMatrixST){
    A = Jg_x;
    for(uint i=0;i<g_x.N;i++) if(!(g_x(i)>0. || lambda(i)>0.)) A[i].setZero();

    arr tmp = comp_A_At(A);
    CHECK(castRowShiftedPackedMatrix(tmp).symmetric==true,"");
    for(uint i=0;i<tmp.d0;i++) tmp(i,0) += 1e-6;

    arr AF = comp_A_x(A, dF_x);
    arr beta;
    lapack_Ainv_b_sym(beta, tmp, AF);

    lambda += lambdaStepsize * (2.*mu*g_x - beta);
  }else{
    //remove zero rows
    for(uint i=0;i<g_x.N;i++) if(g_x(i)>0. || lambda(i)>0.){
      A.append(Jg_x[i]);
      A.reshape(A.N/x.N,x.N);
    }
    arr tmp = A*~A;
    for(uint i=0;i<tmp.d0;i++) tmp(i,i) += 1e-6;
    tmp = inverse_SymPosDef(tmp);
    arr beta = tmp * A;
    //reinsert zero rows
    for(uint i=0;i<g_x.N;i++) if(!(g_x(i)>0. || lambda(i)>0.)){
      beta.insRows(i);
      beta[i].setZero();
    }
    lambda += lambdaStepsize * (2.*mu*g_x - beta*dF_x);
  }

  for(uint i=0;i<g_x.N;i++) if(lambda(i)<0.) lambda(i)=0.;

  if(muInc>1.) mu *= muInc;

  //rescale f
//  if(F_x){
//    double f0 = scalarProduct(lambda - lambdaOld, g_x);
//    for(uint i=0;i<g_x.N;i++){
//      if((lambda(i)>0.  && lambdaOld(i)<=0.) && g_x(i)<=0.) f0 += mu * MT::sqr(g_x(i));
//      if((lambda(i)<=0. && lambdaOld(i)>0. ) && g_x(i)<=0.) f0 -= mu * MT::sqr(g_x(i));
//    }
//    *F_x += f0;
//  }

  if(F_x || &dF_x || &HF_x){
    double fx = fs(dF_x, HF_x, x); //reevaluate gradients and hessian (using buffered info)
    if(F_x) *F_x = fx;
    CHECK(fabs(fx-*F_x)<1e-10,"");
  }
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

const char* MethodName[]={ "NoMethod", "SquaredPenalty", "AugmentedLagrangian", "LogBarrier", "AnyTimeAugmentedLagrangian" };

void optConstrained(arr& x, arr& dual, ConstrainedProblem& P, OptOptions opt){
  UnconstrainedProblem UCP(P);

  //switch on penalty terms
  switch(opt.constrainedMethod){
    case squaredPenalty: UCP.mu=1.;  break;
    case augmentedLag:   UCP.mu=1.;  break;
    case anyTimeAula:    UCP.mu=1.;  break;
    case logBarrier:     UCP.muLB=1.;  break;
    case noMethod: HALT("need to set method before");  break;
  }

  if(opt.verbose>0) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<endl;

  OptNewton newton(x, UCP, opt);
//  OptNewton::StopCriterion res;

  for(uint k=0;;k++){
    if(opt.verbose>0){
      cout <<"***** optConstrained: iteration=" <<k
	   <<" mu=" <<UCP.mu <<" muLB=" <<UCP.muLB;
      if(x.N<5) cout <<" \tlambda=" <<UCP.lambda;
      cout <<endl;
    }

    arr x_old = x;
    if(opt.constrainedMethod==anyTimeAula)
      for(uint l=0;l<2; l++) newton.step();
    else{
      newton.reinit();
      newton.run();
    }

    if(opt.verbose>0){
      cout <<k <<' ' <<newton.evals <<' ' <<"f(x)=" <<UCP.f_x <<" \tcompl=" <<sum(elemWiseMax(UCP.g_x,zeros(UCP.g_x.N,1)));
      if(x.N<5) cout <<" \tx=" <<x;
      cout <<endl;
    }

    //stopping criteron
    if(k>10 && absMax(x_old-x)<opt.stopTolerance){
      cout << " --- optConstrained StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
      break;
    }

    //upate unconstraint problem parameters
    switch(opt.constrainedMethod){
      case squaredPenalty: UCP.mu *= 10;  break;
      case augmentedLag:   UCP.aulaUpdate();  UCP.mu *= 1;  break;
//      case augmentedLag:   UCP.mu *= 1.;
      case anyTimeAula:    UCP.anyTimeAulaUpdate(1., 1., &newton.fx, newton.gx, newton.Hx);  break;
      case logBarrier:     UCP.muLB /= 2;  break;
      case noMethod: HALT("need to set method before");  break;
    }

  }

  if(&dual) dual=UCP.lambda;
}

