#include "optimization.h"

//==============================================================================
//
// UnconstrainedProblem
//
// we define an unconstraint optimization problem from a constrained one
// that can include penalties, log barriers, and augmented lagrangian terms
//

struct UnconstrainedProblem : ScalarFunction{
  /** The VectorFunction F describes the cost function f(x) as well as the constraints g(x)
      concatenated to one vector:
      phi(0) = cost,   phi(1,..,phi.N-1) = constraints */
  ConstrainedProblem &P;
  //-- parameters of the unconstrained meta function F
  double muLB;       ///< log barrier weight
  double mu;         ///< squared penalty weight
  arr lambda;        ///< lagrange multiplier in augmented lagrangian

  //-- buffers to avoid recomputing gradients
  arr x, df_x, Hf_x, g_x, Jg_x;
  double f_x;

  UnconstrainedProblem(ConstrainedProblem &_P):P(_P), muLB(0.), mu(0.) {}

  virtual double fs(arr& dF, arr& HF, const arr& x); ///< the unconstrained meta function F

  void aulaUpdate(double lambdaStepsize=1., arr &x_reeval=NoArr);
  void anyTimeAulaUpdate(double lambdaStepsize=1., double muInc=1., double *F_x=NULL, arr &dF_x=NoArr, arr &HF_x=NoArr);
};


//==============================================================================
//
// PhaseOneProblem
//
// we define a constraint optimization problem that corresponds
// to the phase one problem of another constraint problem
//

struct PhaseOneProblem:ConstrainedProblem{
  ConstrainedProblem &f;

  PhaseOneProblem(ConstrainedProblem &_f):f(_f) {}

  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x);
//  virtual double fs(arr& g, arr& H, const arr& x);
//  virtual void fv(arr& metaPhi, arr& metaJ, const arr& x);
  virtual uint dim_x(){ return f.dim_x()+1; }
  virtual uint dim_g(){ return f.dim_g()+1; }
};


//==============================================================================
//
// Solvers
//

void optConstrained(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt);
