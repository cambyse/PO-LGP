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

#pragma once

#include "optimization.h"

extern const char* MethodName[];

//==============================================================================
//
// UnconstrainedProblem
//
// we define an unconstraint optimization problem from a constrained one
// that can include penalties, log barriers, and augmented lagrangian terms
//

struct UnconstrainedProblem{
  /** The VectorFunction F describes the cost function f(x) as well as the constraints g(x)
      concatenated to one vector:
      phi(0) = cost,   phi(1,..,phi.N-1) = constraints */
  const ConstrainedProblem& P;

  //-- parameters of the unconstrained meta function F
  double muLB;       ///< log barrier weight
  double mu;         ///< squared penalty weight for inequalities g
  double nu;         ///< squared penalty weight for equalities h
  arr lambda;        ///< lagrange multipliers for inequalities g
  arr kappa;         ///< lagrange multiplier for equalities h

  //-- buffers to avoid recomputing gradients
  arr x; ///< point where P was last evaluated
  double f_x; ///< scalar value f(x)
  arr df_x, Hf_x, g_x, Jg_x, h_x, Jh_x; ///< everything else at x

  UnconstrainedProblem(const ConstrainedProblem &P):P(P), muLB(0.), mu(0.), nu(0.) {
    Lag = [this](arr& dL, arr& HL, const arr& x) -> double {
      return this->lagrangian(dL, HL, x);
    };
  }

  double lagrangian(arr& dL, arr& HL, const arr& x); ///< the unconstrained meta function F

  ScalarFunction Lag; ///< the unconstrained problem, typically the (augmented) Lagrangian with given lambda, mu, etc

//  operator const ScalarFunction&(){ return Lag; }

  void aulaUpdate(double lambdaStepsize=1., double muInc=1., double *L_x=NULL, arr &dL_x=NoArr, arr &HL_x=NoArr);
  void anyTimeAulaUpdate(double lambdaStepsize=1., double muInc=1., double *L_x=NULL, arr &dL_x=NoArr, arr &HL_x=NoArr);
  bool anyTimeAulaUpdateStopCriterion(const arr& dL_x);
};


//==============================================================================
//
// PhaseOneProblem
//
// we define a constraint optimization problem that corresponds
// to the phase one problem of another constraint problem
//

struct PhaseOneProblem{
  const ConstrainedProblem &f_orig;
  ConstrainedProblem f_phaseOne;

  PhaseOneProblem(const ConstrainedProblem &f_orig):f_orig(f_orig) {
    f_phaseOne = [this](arr& df, arr& Hf, arr& g, arr& Jg, arr& h, arr& Jh, const arr& x) -> double {
      return this->phase_one(df, Hf, g, Jg, x);
    };
  }
  operator const ConstrainedProblem&(){ return f_phaseOne; }
  double phase_one(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x);
};


//==============================================================================
//
// Solvers
//

uint optConstrained(arr& x, arr &dual, const ConstrainedProblem& P, OptOptions opt=NOOPT);


//==============================================================================
//
// evaluating
//

inline void evaluateConstrainedProblem(const arr& x, ConstrainedProblem& P, std::ostream& os){
  arr g,h;
  double f = P(NoArr, NoArr, g, NoArr, h, NoArr, x);
  os <<"f=" <<f <<" sum([g>0]g)="<<sum(elemWiseMax(g,zeros(g.N,1))) <<" sum(|h|)=" <<sumOfAbs(h) <<endl;
}


