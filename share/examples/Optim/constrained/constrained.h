#include <Optim/optimization.h>

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
  double muLB;       ///< log barrier weight
  double mu;         ///< squared penalty weight
  arr lambda;        ///< lagrange multiplier in augmented lagrangian

  UnconstrainedProblem(ConstrainedProblem &_P):P(_P), muLB(0.), mu(0.) {}

  virtual double fs(arr& g, arr& H, const arr& x); ///< this assumes that only the first entry is costs, rest constraints
//  virtual void fv(arr& y, arr& J, const arr& x); ///< first entries: GaussNewton-type costs, following entries: constraints
  void augmentedLagrangian_LambdaUpdate(const arr& x);
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

  virtual double fs(arr& g, arr& H, const arr& x);
  virtual void fv(arr& metaPhi, arr& metaJ, const arr& x);
  virtual uint get_n(){ return f.get_n()+1; }
  virtual uint get_m(){ return f.get_m()+1; }
};

