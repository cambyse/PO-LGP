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
#include "opt-newton.h"

extern const char* MethodName[];

//==============================================================================
//
// UnconstrainedProblem
//
// we define an unconstraint optimization problem from a constrained one
// that can include lagrange terms, penalties, log barriers, and augmented lagrangian terms
//

struct UnconstrainedProblemMix:ScalarFunction{
  /** The VectorFunction F describes the cost function f(x) as well as the constraints g(x)
      concatenated to one vector:
      phi(0) = cost,   phi(1,..,phi.N-1) = constraints */
  ConstrainedProblemMix P;

  //-- parameters of the unconstrained scalar function
  double muLB;       ///< log barrier weight
  double mu;         ///< squared penalty weight for inequalities g
  double nu;         ///< squared penalty weight for equalities h
  arr lambda;        ///< lagrange multipliers for inequalities g and equalities h

  //-- buffers to avoid recomputing gradients
  arr x;          ///< point where P was last evaluated
  arr phi_x, J_x; ///< everything else at x
  TermTypeA tt_x; ///< everything else at x

  UnconstrainedProblemMix(const ConstrainedProblemMix &P, ConstrainedMethodType method, arr& lambdaInit=NoArr);

  double lagrangian(arr& dL, arr& HL, const arr& x); ///< the unconstrained scalar function F

  double get_costs();            ///< info on the terms from last call
  double get_sumOfGviolations(); ///< info on the terms from last call
  double get_sumOfHviolations(); ///< info on the terms from last call

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
  const ConstrainedProblemMix &f_orig;
  ConstrainedProblemMix f_phaseOne;

  PhaseOneProblem(const ConstrainedProblemMix &f_orig):f_orig(f_orig) {
    f_phaseOne = [this](arr& phi, arr& J, TermTypeA& tt, const arr& x) -> void {
      return this->phase_one(phi, J, tt, x);
    };
  }
  operator const ConstrainedProblemMix&(){ return f_phaseOne; }
  void phase_one(arr& phi, arr& J, TermTypeA& tt, const arr& x);
};


//==============================================================================
//
// Solvers
//

uint optConstrainedMix(arr& x, arr &dual, const ConstrainedProblemMix& P, OptOptions opt=NOOPT);

struct OptConstrained{
  UnconstrainedProblemMix UCP;
  OptNewton newton;
  arr &dual;
  OptOptions opt;
  uint its;
  ofstream fil;

  OptConstrained(arr& x, arr &dual, const ConstrainedProblemMix& P, OptOptions opt=NOOPT);
  ~OptConstrained();
  bool step();
  void run();
//  void reinit();
};


//==============================================================================
//
// evaluating
//

inline void evaluateConstrainedProblem(const arr& x, ConstrainedProblemMix& P, std::ostream& os){
  arr phi_x;
  TermTypeA tt_x;
  P(phi_x, NoArr, tt_x, x);
  double Ef=0., Eh=0., Eg=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==sumOfSqrTT) Ef += MT::sqr(phi_x(i));
    if(tt_x(i)==ineqTT && phi_x(i)>0.) Eg += phi_x(i);
    if(tt_x(i)==eqTT) Eh += fabs(phi_x(i));
  }
  os <<"f=" <<Ef <<" sum([g>0]g)="<<Eg <<" sum(|h|)=" <<Eh <<endl;
}


