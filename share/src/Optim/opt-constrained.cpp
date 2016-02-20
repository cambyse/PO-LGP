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

//==============================================================================

UnconstrainedProblem::UnconstrainedProblem(const ConstrainedProblem& P, OptOptions opt, arr& lambdaInit)
  : P(P), muLB(0.), mu(0.), nu(0.) {
  ScalarFunction::operator=( [this](arr& dL, arr& HL, const arr& x) -> double {
    return this->lagrangian(dL, HL, x);
  } );

  //switch on penalty terms
  nu=opt.muInit;
  switch(opt.constrainedMethod){
    case squaredPenalty: mu=opt.muInit;  break;
    case augmentedLag:   mu=opt.muInit;  break;
    case anyTimeAula:    mu=opt.muInit;  break;
    case logBarrier:     muLB=opt.muLBInit;  break;
    case squaredPenaltyFixed: mu=opt.muInit;  break;
    case noMethod: HALT("need to set method before");  break;
  }

  if(&lambdaInit) lambda = lambdaInit;
}

double UnconstrainedProblem::lagrangian(arr& dL, arr& HL, const arr& _x){
  //-- evaluate constrained problem and buffer
  if(_x!=x){
    x=_x;
    P(phi_x, J_x, H_x, tt_x, x);
  }else{ //we evaluated this before - use buffered values; the meta F is still recomputed as (dual) parameters might have changed
  }
  CHECK_EQ(phi_x.N, J_x.d0, "Jacobian size inconsistent");
  CHECK_EQ(phi_x.N, tt_x.N, "termType array size inconsistent");

  //-- construct unconstrained problem
  //precompute I_lambda_x
  boolA I_lambda_x(phi_x.N);
  if(phi_x.N) I_lambda_x = false;
  if(mu)       for(uint i=0;i<phi_x.N;i++) if(tt_x(i)==ineqTT) I_lambda_x(i) = (phi_x(i)>0. || (lambda.N && lambda(i)>0.));

  double L=0.; //L value
  for(uint i=0;i<phi_x.N;i++){
    if(            tt_x(i)==fTT                    ) L += phi_x(i);                // direct cost term
    if(            tt_x(i)==sumOfSqrTT             ) L += mlr::sqr(phi_x(i));       // sumOfSqr term
    if(muLB     && tt_x(i)==ineqTT                 ){ if(phi_x(i)>0.) return NAN;  L -= muLB * ::log(-phi_x(i)); } //log barrier, check feasibility
    if(mu       && tt_x(i)==ineqTT && I_lambda_x(i)) L += mu * mlr::sqr(phi_x(i));  //g-penalty
    if(lambda.N && tt_x(i)==ineqTT && lambda(i)>0. ) L += lambda(i) * phi_x(i);    //g-lagrange terms
    if(nu       && tt_x(i)==eqTT                   ) L += nu * mlr::sqr(phi_x(i));  //h-penalty
    if(lambda.N && tt_x(i)==eqTT                   ) L += lambda(i) * phi_x(i);    //h-lagrange terms
  }

  if(&dL){ //L gradient
    arr coeff=zeros(phi_x.N);
    for(uint i=0;i<phi_x.N;i++){
      if(            tt_x(i)==fTT                    ) coeff(i) += 1.;              // direct cost term
      if(            tt_x(i)==sumOfSqrTT             ) coeff(i) += 2.* phi_x(i);    // sumOfSqr terms
      if(muLB     && tt_x(i)==ineqTT                 ) coeff(i) -= (muLB/phi_x(i)); //log barrier, check feasibility
      if(mu       && tt_x(i)==ineqTT && I_lambda_x(i)) coeff(i) += 2.*mu*phi_x(i);  //g-penalty
      if(lambda.N && tt_x(i)==ineqTT && lambda(i)>0. ) coeff(i) += lambda(i);       //g-lagrange terms
      if(nu       && tt_x(i)==eqTT                   ) coeff(i) += 2.*nu*phi_x(i);  //h-penalty
      if(lambda.N && tt_x(i)==eqTT                   ) coeff(i) += lambda(i);       //h-lagrange terms
    }
    dL = comp_At_x(J_x, coeff);
    dL.reshape(x.N);
  }

  if(&HL){ //L hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
    arr coeff=zeros(phi_x.N);
    int fterm=-1;
    for(uint i=0;i<phi_x.N;i++){
      if(            tt_x(i)==fTT){ if(fterm!=-1) HALT("There must only be 1 f-term (in the current implementation)");  fterm=i; }
      if(            tt_x(i)==sumOfSqrTT             ) coeff(i) += 2.;      // sumOfSqr terms
      if(muLB     && tt_x(i)==ineqTT                 ) coeff(i) += (muLB/mlr::sqr(phi_x(i)));  //log barrier, check feasibility
      if(mu       && tt_x(i)==ineqTT && I_lambda_x(i)) coeff(i) += 2.*mu;   //g-penalty
      if(nu       && tt_x(i)==eqTT                   ) coeff(i) += 2.*nu;   //h-penalty
    }
    arr tmp = J_x;
    for(uint i=0;i<phi_x.N;i++) tmp[i]() *= sqrt(coeff(i));
    HL = comp_At_A(tmp); //Gauss-Newton type!

    if(fterm!=-1){ //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
      HL += H_x;
    }

    if(!HL.special) HL.reshape(x.N,x.N);
  }

  return L;
}

double UnconstrainedProblem::get_costs(){
  double S=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==fTT) S += phi_x(i);
    if(tt_x(i)==sumOfSqrTT) S += mlr::sqr(phi_x(i));
  }
  return S;
}

double UnconstrainedProblem::get_sumOfGviolations(){
  double S=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==ineqTT && phi_x(i)>0.) S += phi_x(i);
  }
  return S;
}

double UnconstrainedProblem::get_sumOfHviolations(){
  double S=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==eqTT) S += fabs(phi_x(i));
  }
  return S;
}

uint UnconstrainedProblem::get_dimOfType(const TermType& tt){
  uint d=0;
  for(uint i=0;i<tt_x.N;i++) if(tt_x(i)==tt) d++;
  return d;
}

void UnconstrainedProblem::aulaUpdate(bool anyTimeVariant, double lambdaStepsize, double muInc, double *L_x, arr& dL_x, arr& HL_x){
  if(!lambda.N) lambda=zeros(phi_x.N);

  //-- lambda update
  for(uint i=0;i<lambda.N;i++){
    if(tt_x(i)==eqTT  )  lambda(i) += (lambdaStepsize * 2.*nu)*phi_x(i);
    if(tt_x(i)==ineqTT)  lambda(i) += (lambdaStepsize * 2.*mu)*phi_x(i);
    if(tt_x(i)==ineqTT && lambda(i)<0.) lambda(i)=0.;  //bound clipping
  }

  if(anyTimeVariant){
    //collect gradients of active constraints
    arr A;
    RowShiftedPackedMatrix *Aaux, *Jaux;
    if(J_x.special==arr::RowShiftedPackedMatrixST){
      Aaux = auxRowShifted(A, 0, J_x.d1, x.N);
      Jaux = &castRowShiftedPackedMatrix(J_x);
    }
    //append rows of J_x to A if constraint is active
    for(uint i=0;i<lambda.N;i++){
      if( (tt_x(i)==eqTT) ||
          (tt_x(i)==ineqTT && (phi_x(i)>0. || lambda(i)>0.)) ){
        A.append(J_x[i]);
        A.reshape(A.N/J_x.d1,J_x.d1);
        if(J_x.special==arr::RowShiftedPackedMatrixST)
          Aaux->rowShift.append(Jaux->rowShift(i));
      }
    }
    if(A.d0>0){
      arr tmp = comp_A_At(A);
      addDiag(tmp, 1e-6);
      //    if(J_x.special==arr::RowShiftedPackedMatrixST){
      //      CHECK_EQ(castRowShiftedPackedMatrix(tmp).symmetric,true,"");
      //      for(uint i=0;i<tmp.d0;i++) tmp(i,0) += 1e-6;
      //    }else{
      //      for(uint i=0;i<tmp.d0;i++) tmp(i,i) += 1e-6;
      //    }

      arr AdL = comp_A_x(A, dL_x);
      arr beta;
      beta = lapack_Ainv_b_sym(tmp, AdL);
      //reinsert zero rows
      for(uint i=0;i<lambda.N;i++){
        if(! ( (tt_x(i)==eqTT) ||
               (tt_x(i)==ineqTT && (phi_x(i)>0. || lambda(i)>0.)) ) ){
          beta.insert(i,0.);
        }
      }
      lambda -= lambdaStepsize * beta;
      //bound clipping
      for(uint i=0;i<lambda.N;i++) if(lambda(i)<0.) lambda(i)=0.;
    }
  }

  //-- adapt mu as well?
  if(muInc>1. && mu<1e6) mu *= muInc;
  if(muInc>1. && nu<1e6) nu *= muInc;

  //-- recompute the Lagrangian with the new parameters (its current value, gradient & hessian)
  if(L_x || &dL_x || &HL_x){
    double L = lagrangian(dL_x, HL_x, x); //reevaluate gradients and hessian (using buffered info)
    if(L_x) *L_x = L;
  }
}

//==============================================================================
//
// PhaseOneProblem
//


void PhaseOneProblem::phase_one(arr& meta_phi, arr& meta_J, arr& meta_H, TermTypeA& tt, const arr& x){
  NIY;
  arr g, Jg;
//  f_orig(NoArr, NoArr, g, (&meta_Jg?Jg:NoArr), x.sub(0,-2)); //the underlying problem only receives a x.N-1 dimensional x

  // meta_g.resize(g.N+1);
  // meta_g(0) = x.last();                                       //cost
  // for(uint i=0;i<g.N;i++) meta_g(i) = g(i)-x.last();  //slack constraints
  // meta_g.last() = -x.last();                                  //last constraint

  // if(&meta_Jg){
  //   meta_Jg.resize(meta_g.N, x.N);  meta_Jg.setZero();
  //   meta_Jg(0,x.N-1) = 1.; //cost
  //   for(uint i=0;i<g.N;i++) for(uint j=0;j<x.N-1;j++) meta_Jg(i,j) = Jg(i,j);
  //   for(uint i=0;i<g.N;i++) meta_Jg(i,x.N-1) = -1.;
  //   meta_Jg(g.N, x.N-1) = -1.;
  // }
}


//==============================================================================
//
// Solvers
//

const char* MethodName[]={ "NoMethod", "SquaredPenalty", "AugmentedLagrangian", "LogBarrier", "AnyTimeAugmentedLagrangian", "SquaredPenaltyFixed"};

uint optConstrained(arr& x, arr& dual, const ConstrainedProblem& P, OptOptions opt){
  return OptConstrained(x, dual, P, opt).run();
}

//==============================================================================

OptConstrained::OptConstrained(arr& x, arr &dual, const ConstrainedProblem& P, OptOptions opt)
  : UCP(P, opt, dual), newton(x, UCP, opt), dual(dual), opt(opt), its(0), earlyPhase(true){

  fil.open(STRING("z."<<MethodName[opt.constrainedMethod]));

  if(opt.verbose>0) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<endl;
}

bool OptConstrained::step(){
  fil <<its <<' ' <<newton.evals <<' ' <<UCP.get_costs() <<' ' <<UCP.get_sumOfGviolations() <<' ' <<UCP.get_sumOfHviolations() <<endl;

  if(opt.verbose>0){
    cout <<"** optConstr. it=" <<its
         <<(earlyPhase?'e':'l')
         <<" mu=" <<UCP.mu <<" nu=" <<UCP.nu <<" muLB=" <<UCP.muLB;
    if(newton.x.N<5) cout <<" \tlambda=" <<UCP.lambda;
    cout <<endl;
  }

  arr x_old = newton.x;

  if(opt.constrainedMethod==squaredPenaltyFixed){
    newton.run();
  }else{
    double stopTol = newton.o.stopTolerance;
    newton.o.stopTolerance *= (earlyPhase?10.:2.);
    if(opt.constrainedMethod==anyTimeAula)  newton.run(20);
    else                                    newton.run();
    newton.o.stopTolerance = stopTol;
  }

  if(opt.verbose>0){
    cout <<"** optConstr. it=" <<its
         <<(earlyPhase?'e':'l')
         <<' ' <<newton.evals
         <<" f(x)=" <<UCP.get_costs()
         <<" \tg_compl=" <<UCP.get_sumOfGviolations()
         <<" \th_compl=" <<UCP.get_sumOfHviolations()
         <<" \t|x-x'|=" <<absMax(x_old-newton.x);
    if(newton.x.N<5) cout <<" \tx=" <<newton.x;
    cout <<endl;
  }

  //check for squaredPenaltyFixed method
  if(opt.constrainedMethod==squaredPenaltyFixed){
    if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }

  //check for no constraints
  if(UCP.get_dimOfType(ineqTT) + UCP.get_dimOfType(eqTT) == 0){
    if(opt.verbose>0) cout <<"** optConstr. NO CONSTRAINTS -> run Newton againg and stop" <<endl;
    newton.run();
    return true;
  }

  //stopping criteron
  if(its>=2 && absMax(x_old-newton.x)<opt.stopTolerance){
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
    if(earlyPhase) earlyPhase=false;
    else{
      if(opt.stopGTolerance<0.
         || UCP.get_sumOfGviolations() + UCP.get_sumOfHviolations() < opt.stopGTolerance)
        return true;
     }
  }

  //upate Lagrange parameters
  switch(opt.constrainedMethod){
    case squaredPenalty: UCP.mu *= 10.;  break;
    case augmentedLag:   UCP.aulaUpdate(false, 1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
    case anyTimeAula:    UCP.aulaUpdate(true,  1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
    case logBarrier:     UCP.muLB /= 2.;  break;
    case squaredPenaltyFixed: HALT("you should not be here"); break;
    case noMethod: HALT("need to set method before");  break;
  }

  if(&dual) dual=UCP.lambda;

  its++;

  return false;
}

uint OptConstrained::run(){
  earlyPhase=true;
  while(!step());
  return newton.evals;
}

OptConstrained::~OptConstrained(){
}


