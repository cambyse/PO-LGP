#ifndef MT_optimization_h
#define MT_optimization_h

#include "array.h"
#include "util.h"

// function evaluation counter (used only for performance meassurements, global for simplicity)
extern uint eval_cost;

//===========================================================================
//
// return types
//

/// return type for a function that returns a square potential $f(x) = x^T A x - 2 a^T x + c
struct     SqrPotential { arr A, a;          double c; };
/// return type for a function that returns a square potential $f(x,y) = [x,y]^T [A,C; C^T,B] [x,y] - 2 [a,b]^T [x,y] + c$
struct PairSqrPotential { arr A, B, C, a, b; double c; };

extern arr& NoGrad;
extern SqrPotential& NoPot;
extern PairSqrPotential& NoPairPot;


//===========================================================================
//
// (cost) function types
//

/// A scalar function $y = f(x)$, if @grad@ is not NoGrad, gradient is returned
struct ScalarFunction { virtual double fs(arr& grad, const arr& x) = 0; };

/// A vector function $y = f(x)$, if @J@ is not NoGrad, Jacobian is returned
/// This also implies an optimization problem $\hat f(y) = y^T(x) y(x)$ of (iterated)
/// Gauss-Newton type where the Hessian is approximated by J^T J
struct VectorFunction { virtual void   fv(arr& y, arr& J, const arr& x) = 0; };

/// A scalar function $y = f(x)$, if @S@ is non-NULL, local quadratic approximation is returned
/// This also implies an optimization problem of (iterated) Newton
/// type with the given Hessian
struct QuadraticFunction { virtual double fq(SqrPotential& S, const arr& x) = 0; };


/// Given a chain $x_{0:T}$ of variables, implies a cost function
/// $f(x) = \sum_{i=0}^T f_i(x_i)^T f_i(x_i) + \sum_{i=1}^T f_{ij}(x_i,x_j)^T f_{ij}(x_i,x_j)$
/// and we can access local Jacobians of f_i and f_{ij}
struct VectorChainFunction {
  uint T;
  virtual void fvi(arr& y, arr& J, uint i, const arr& x_i) = 0;
  virtual void fvij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

/// Given a chain $x_{0:T}$ of variables, implies a cost function
/// $f(x) = \sum_{i=0}^T f_i(x_i) + \sum_{i=1}^T f_{ij}(x_i,x_j)$
/// and we can access local SqrPotential approximations of f_i and f_{ij}
struct SqrChainFunction {
  uint T;
  virtual double fqi(SqrPotential& S, uint i, const arr& x_i) = 0;
  virtual double fqij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

/*
struct VectorGraphFunction {
  virtual uintA edges() = 0;
  virtual double fi (arr* grad, uint i, const arr& x_i) = 0;
  virtual double fij(arr* gradi, arr* gradj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
  double f_total(const arr& X);
};*/


//===========================================================================
//
// checks, evaluation and converters
//

bool checkGradient(ScalarFunction &f, const arr& x, double tolerance);
bool checkJacobian(VectorFunction &f, const arr& x, double tolerance);
bool checkHessian(QuadraticFunction &f, const arr& x, double tolerance); //TODO! NIY!

//these directly simply evaluate squared potentials at some point
double evaluateSP(const SqrPotential& S, const arr& x);
double evaluatePSP(const PairSqrPotential& S, const arr& x, const arr& y);
double evaluateCSP(const MT::Array<SqrPotential>& fi, const MT::Array<PairSqrPotential>& fij, const arr& x);

//these actually call the functions (->query cost) to evalute them at some point
double evaluateSF(ScalarFunction& f, const arr& x);
double evaluateVF(VectorFunction& f, const arr& x);
double evaluateVCF(VectorChainFunction& f, const arr& x);
double evaluateQCF(SqrChainFunction& f, const arr& x);

/// converts a VectorChainFunction to any of a ScalarFunction, VectorFunction or SqrChainFunction
struct conv_VectorChainFunction:ScalarFunction,VectorFunction,SqrChainFunction {
  VectorChainFunction *f;
  conv_VectorChainFunction(VectorChainFunction& _f);
  virtual double fs(arr& grad, const arr& x);                  //to a ScalarFunction
  virtual void   fv(arr& y, arr& J, const arr& x);             //to a VectorFunction
  virtual double fqi(SqrPotential& S, uint i, const arr& x_i);  //to a SqrChainFunction
  virtual double fqij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j);
};


//===========================================================================
//
// optimization methods
//

struct optOptions {
  double *fmin_return;
  double stopTolerance;
  uint   stopEvals;
  uint   stopIters;
  double initialDamping;
  double initStep;
  double minStep;
  double maxStep;
  bool clampInitialState;
  uint verbose;
  optOptions();
};

extern optOptions globalOptOptions;
#define OPT1(a) (globalOptOptions.a, globalOptOptions)
#define OPT2(a,b) (globalOptOptions.a, globalOptOptions.b, globalOptOptions)
#define OPT3(a,b,c) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions)
#define OPT4(a,b,c,d) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions)
#define OPT5(a,b,c,d,e) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions.e, globalOptOptions)
#define OPT6(a,b,c,d,e,f) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions.e, globalOptOptions.f, globalOptOptions)

/// minimizes f(x) = phi(x)^T phi(x) using the Jacobian of phi
/// the optional _user arguments specify, if f has already been evaluated at x (another initial evaluation is then omitted
/// to increase performance) and the evaluation of the returned x is also returned
uint optGaussNewton(arr& x, VectorFunction& phi, optOptions opt, arr *fx_user=NULL, arr *Jx_user=NULL);

/// minimizes f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)
/// the optional _user arguments specify, if f has already been evaluated at x (another initial evaluation is then omitted
/// to increase performance) and the evaluation of the returned x is also returned
uint optNewton(arr& x, QuadraticFunction& f, optOptions opt, double *fx_user=NULL, SqrPotential *Sx_user=NULL);

/// minimizes f(x)
uint optRprop(arr& x, ScalarFunction& f, optOptions opt);

/// minimizes f(x)
uint optGradDescent(arr& x, ScalarFunction& f, optOptions opt);

uint optDynamicProgramming(arr& x, SqrChainFunction& f, optOptions opt);

uint optNodewise(arr& x, VectorChainFunction& f, optOptions opt);

uint optMinSumGaussNewton(arr& x, SqrChainFunction& f, optOptions opt);


//===========================================================================
//
// Rprop
//

/*! Rprop, a fast gradient-based minimization */
struct Rprop {
  struct sRprop *s;
  Rprop();
  void init(double initialStepSize=1., double minStepSize=1e-6, double maxStepSize=50.);
  bool step(arr& x, ScalarFunction& f);
  uint loop(arr& x, ScalarFunction& f, double *fmin_return=NULL, double stoppingTolerance=1e-2, double initialStepSize=1., uint maxIterations=1000, uint verbose=0);
};


//===========================================================================
//
// named parameters (options)
//

optOptions* accessOpt();
optOptions& deaccessOpt(optOptions*);



#ifdef  MT_IMPLEMENTATION
#  include "optimization.cpp"
#endif

#endif
