/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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


#ifndef MT_optimization_h
#define MT_optimization_h

#include "array.h"
#include "util.h"

// function evaluation counter (used only for performance meassurements, global for simplicity)
extern uint eval_cost;


//===========================================================================
//
// functions that imply (optimization) problems
//

//-- return types
struct     SqrPotential { arr A, a;          double c; };
struct PairSqrPotential { arr A, B, C, a, b; double c; };
extern arr& NoGrad;
extern SqrPotential& NoPot;
extern PairSqrPotential& NoPairPot;

struct ScalarFunction { virtual double fs(arr& g, arr& H, const arr& x) const = 0; };

struct VectorFunction { virtual void   fv(arr& y, arr& J, const arr& x) const = 0; };

struct QuadraticFunction { virtual double fq(SqrPotential& S, const arr& x) const = 0; };

struct VectorChainFunction {
  virtual uint get_T() = 0;
  virtual void fv_i(arr& y, arr& J, uint i, const arr& x_i) = 0;
  virtual void fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

struct QuadraticChainFunction {
  virtual uint get_T() = 0;
  virtual double fq_i(SqrPotential& S, uint i, const arr& x_i) = 0;
  virtual double fq_ij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

struct KOrderMarkovFunction {
  virtual void phi_t(arr& phi, arr& J, uint t, const arr& x_bar) = 0;

  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() = 0;
  virtual uint get_k() = 0;
  virtual uint get_n() = 0; //the dimensionality of $x_t$
  virtual uint get_m(uint t) = 0; //the dimensionality of $\phi_t$

  //optional: kernel costs
  virtual bool hasKernel(){ return false; }
  virtual double kernel(uint t0,uint t1){ NIY; }
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

struct Convert{
  struct sConvert* s;
  Convert(ScalarFunction&);
  Convert(VectorFunction&);
  Convert(QuadraticFunction&);
  Convert(VectorChainFunction&);
  Convert(QuadraticChainFunction&);
  Convert(KOrderMarkovFunction&);
  Convert(struct ControlledSystem&);
  ~Convert();
  operator ScalarFunction&();
  operator VectorFunction&();
  operator VectorChainFunction&();
  operator QuadraticChainFunction&();
  operator KOrderMarkovFunction&();
};


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
double evaluateQCF(QuadraticChainFunction& f, const arr& x);


//===========================================================================
//
// optimization methods
//

struct optOptions {
  double *fmin_return;
  double stopTolerance;
  uint   stopEvals;
  uint   stopIters;
  double useAdaptiveDamping;
  double initStep;
  double minStep;
  double maxStep;
  bool clampInitialState;
  uint verbose;
  optOptions();
};

extern optOptions globalOptOptions;
#define OPT0() (globalOptOptions)
#define OPT1(a) (globalOptOptions.a, globalOptOptions)
#define OPT2(a,b) (globalOptOptions.a, globalOptOptions.b, globalOptOptions)
#define OPT3(a,b,c) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions)
#define OPT4(a,b,c,d) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions)
#define OPT5(a,b,c,d,e) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions.e, globalOptOptions)
#define OPT6(a,b,c,d,e,f) (globalOptOptions.a, globalOptOptions.b, globalOptOptions.c, globalOptOptions.d, globalOptOptions.e, globalOptOptions.f, globalOptOptions)

/// minimizes f(x) = phi(x)^T phi(x) using the Jacobian of phi
/// the optional _user arguments specify, if f has already been evaluated at x (another initial evaluation is then omitted
/// to increase performance) and the evaluation of the returned x is also returned
uint optGaussNewton(arr& x, VectorFunction& phi, optOptions opt, arr *addRegularizer=NULL, arr *fx_user=NULL, arr *Jx_user=NULL);

/// minimizes f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)
/// the optional _user arguments specify, if f has already been evaluated at x (another initial evaluation is then omitted
/// to increase performance) and the evaluation of the returned x is also returned
uint optNewton(arr& x, QuadraticFunction& f, optOptions opt, double *fx_user=NULL, SqrPotential *Sx_user=NULL);

/// minimizes f(x)
uint optRprop(arr& x, ScalarFunction& f, optOptions opt);

/// minimizes f(x)
uint optGradDescent(arr& x, ScalarFunction& f, optOptions opt);

uint optDynamicProgramming(arr& x, QuadraticChainFunction& f, optOptions opt);

uint optNodewise(arr& x, VectorChainFunction& f, optOptions opt);

uint optMinSumGaussNewton(arr& x, QuadraticChainFunction& f, optOptions opt);


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
