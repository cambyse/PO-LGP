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

#include "opt-constrained.h"
#include "opt-newton.h"

//==============================================================================
//
// UnconstrainedProblem
//

double I_lambda_x(uint i, arr& lambda, arr& g){
  if(g(i)>0. || (lambda.N && lambda(i)>0.)) return 1.;
  return 0.;
}

double UnconstrainedProblem::fs(arr& dL, arr& HL, const arr& _x){
  //-- evaluate constrained problem and buffer
  if(_x!=x){
    x=_x;
    f_x = P.fc(df_x, Hf_x, g_x, Jg_x, x);
    CHECK(P.dim_g()==g_x.N,"this conversion requires phi.N to be m-dimensional");
  }else{ //we evaluated this before - use buffered values; the meta F is still recomputed as (dual) parameters might have changed
    if(&dL) CHECK(df_x.N && Jg_x.nd,"");
    if(&HL) CHECK(Hf_x.N && Jg_x.nd,"");
  }

  //  cout <<"g= " <<g_x <<" lambda= " <<lambda <<endl;

  //-- construct unconstrained problem
  //precompute I_lambda_x
  boolA I_lambda_x(g_x.N);
  if(mu)       for(uint i=0;i<g_x.N;i++) I_lambda_x(i) = (g_x(i)>0. || (lambda.N && lambda(i)>0.));

  //L value
  double L=f_x;
  if(muLB)     for(uint i=0;i<g_x.N;i++){ if(g_x(i)>0.) return NAN;  L -= muLB * ::log(-g_x(i)); } //log barrier, check feasibility
  if(mu)       for(uint i=0;i<g_x.N;i++) if(I_lambda_x(i)) L += mu * MT::sqr(g_x(i));  //penalty
  if(lambda.N) for(uint i=0;i<g_x.N;i++) if(lambda(i)>0.) L += lambda(i) * g_x(i);  //augments

  if(&dL){ //L gradient
    dL=df_x;
    arr coeff(Jg_x.d0); coeff.setZero();
    if(muLB)     for(uint i=0;i<g_x.N;i++) coeff(i) -= (muLB/g_x(i));  //log barrier
    if(mu)       for(uint i=0;i<g_x.N;i++) if(I_lambda_x(i)) coeff(i) += 2.*mu*g_x(i);  //penalty
    if(lambda.N) for(uint i=0;i<g_x.N;i++) if(lambda(i)>0.) coeff(i) += lambda(i);  //augments
    dL += comp_At_x(Jg_x, coeff);
    dL.reshape(x.N);
  }

  if(&HL){ //L hessian
    HL=Hf_x;
    /// the 2.*Jg_x^T Jg_x terms are considered as in Gauss-Newton type; no real Hg used
    arr coeff(Jg_x.d0); coeff.setZero();
    if(muLB)     for(uint i=0;i<g_x.N;i++) coeff(i) += (muLB/MT::sqr(g_x(i)));  //log barrier
    if(mu)       for(uint i=0;i<g_x.N;i++) if(I_lambda_x(i)) coeff(i) += 2.*mu;  //penalty
    //if(lambda.N) for(uint i=0;i<g_x.N;i++) if(lambda(i)>0.) coeff(i) += 0.; //augments -> evaluates to zero
    arr tmp = Jg_x;
    for(uint i=0;i<g_x.N;i++) tmp[i]() *= sqrt(coeff(i));
    HL += comp_At_A(tmp); //Gauss-Newton type!
    if(!HL.special) HL.reshape(x.N,x.N);
  }

  return L;
}

void UnconstrainedProblem::aulaUpdate(double lambdaStepsize, double muInc, double *L_x, arr &dL_x, arr &HL_x){
  if(!lambda.N){ lambda.resize(g_x.N); lambda.setZero(); }

  lambda += (lambdaStepsize * 2.*mu)*g_x;
  for(uint i=0;i<lambda.N;i++) if(lambda(i)<0.) lambda(i)=0.;

  //-- adapt mu as well
  if(muInc>1.) mu *= muInc;

  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || &dL_x || &HL_x){
    double L = fs(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
  }
}

bool UnconstrainedProblem::anyTimeAulaUpdateStopCriterion(const arr& dL_x){
//  cout <<"checking gradient norms: dL_x =" <<length(dL_x) <<" df_x=" <<length(df_x) <<endl;
  if(length(dL_x) < .1 * length(df_x)) return true;
  return false;
}

void UnconstrainedProblem::anyTimeAulaUpdate(double lambdaStepsize, double muInc, double *L_x, arr& dL_x, arr& HL_x){
  if(!lambda.N){ lambda.resize(g_x.N); lambda.setZero(); }

  lambda += (lambdaStepsize * 2.*mu)*g_x;
  //bound clipping
  for(uint i=0;i<g_x.N;i++) if(lambda(i)<0.) lambda(i)=0.;

  //collect gradients of active constraints
  arr A;
  RowShiftedPackedMatrix *Aaux, *Jgaux;
  if(Jg_x.special==arr::RowShiftedPackedMatrixST){
    Aaux = auxRowShifted(A, 0, Jg_x.d1, x.N);
    Jgaux = &castRowShiftedPackedMatrix(Jg_x);
  }
  //append rows of Jg_x to A if constraint is active
  for(uint i=0;i<g_x.N;i++) if(g_x(i)>0. || lambda(i)>0.){
    A.append(Jg_x[i]);
    A.reshape(A.N/Jg_x.d1,Jg_x.d1);
    if(Jg_x.special==arr::RowShiftedPackedMatrixST)
      Aaux->rowShift.append(Jgaux->rowShift(i));
  }
  if(A.d0>0){
    arr tmp = comp_A_At(A);
    if(Jg_x.special==arr::RowShiftedPackedMatrixST){
      CHECK(castRowShiftedPackedMatrix(tmp).symmetric==true,"");
      for(uint i=0;i<tmp.d0;i++) tmp(i,0) += 1e-6;
    }else{
      for(uint i=0;i<tmp.d0;i++) tmp(i,i) += 1e-6;
    }

    arr AdL = comp_A_x(A, dL_x);
    arr beta;
    beta = lapack_Ainv_b_sym(tmp, AdL);
    //reinsert zero rows
    for(uint i=0;i<g_x.N;i++) if(!(g_x(i)>0. || lambda(i)>0.)){
      beta.insert(i,0.);
    }
    lambda -= lambdaStepsize * beta;
    //bound clipping
    for(uint i=0;i<g_x.N;i++) if(lambda(i)<0.) lambda(i)=0.;
  }

  //-- adapt mu as well
  if(muInc>1.) mu *= muInc;

  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || &dL_x || &HL_x){
    double L = fs(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
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

uint optConstrained(arr& x, arr& dual, ConstrainedProblem& P, OptOptions opt){

  ofstream fil(STRING("z."<<MethodName[opt.constrainedMethod]));

  UnconstrainedProblem UCP(P);

  uint stopTolInc;

  //switch on penalty terms
  switch(opt.constrainedMethod){
    case squaredPenalty: UCP.mu=1.;  break;
    case augmentedLag:   UCP.mu=1.;  break;
    case anyTimeAula:    UCP.mu=1.;  stopTolInc=MT::getParameter("/opt/optConstrained/anyTimeAulaStopTolInc",2.); break;
    case logBarrier:     UCP.muLB=.1;  break;
    case noMethod: HALT("need to set method before");  break;
  }

  if(opt.verbose>0) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<endl;

  OptNewton newton(x, UCP, opt);

  for(uint k=0;;k++){
    fil <<k <<' ' <<newton.evals <<' ' <<UCP.f_x <<' ' <<sum(elemWiseMax(UCP.g_x,zeros(UCP.g_x.N,1))) <<endl;

    if(opt.verbose>0){
      cout <<"***** optConstrained: iteration=" <<k
	   <<" mu=" <<UCP.mu <<" muLB=" <<UCP.muLB;
      if(x.N<5) cout <<" \tlambda=" <<UCP.lambda;
      cout <<endl;
    }

    arr x_old = x;
    if(opt.constrainedMethod==anyTimeAula){
      //decide yourselve on when to stop iterating Newton steps
      double stopTol = newton.o.stopTolerance;
      newton.o.stopTolerance*=10.;
#if 0
      for(uint l=0;l<20; l++){
        OptNewton::StopCriterion res = newton.step();
        if(res>=OptNewton::stopCrit1) break;
//        if(UCP.anyTimeAulaUpdateStopCriterion(newton.gx)) break;
        newton.o.stopTolerance*=stopTolInc;
      }
#else
      newton.run();
#endif
      newton.o.stopTolerance = stopTol;
    }else{
      //use standard 'run()' to iterate Newton steps
      double stopTol = newton.o.stopTolerance;
      newton.o.stopTolerance*=10.;
      newton.run();
      newton.o.stopTolerance = stopTol;
    }

    if(opt.verbose>0){
      cout <<k <<' ' <<newton.evals <<' ' <<"f(x)=" <<UCP.f_x <<" \tcompl=" <<sum(elemWiseMax(UCP.g_x,zeros(UCP.g_x.N,1)));
      if(x.N<5) cout <<" \tx=" <<x;
      cout <<endl;
    }

    //stopping criteron
    if(k>10 && absMax(x_old-x)<opt.stopTolerance){
      if(opt.verbose>0) cout << " --- optConstrained StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
      break;
    }

    //upate unconstraint problem parameters
    switch(opt.constrainedMethod){
      case squaredPenalty: UCP.mu *= 10;  break;
      case augmentedLag:   UCP.aulaUpdate(1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
      case anyTimeAula:    UCP.anyTimeAulaUpdate(1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
      case logBarrier:     UCP.muLB /= 2;  break;
      case noMethod: HALT("need to set method before");  break;
    }

  }
  fil.close();
  if(&dual) dual=UCP.lambda;

  return newton.evals;
}

