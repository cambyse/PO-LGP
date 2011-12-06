#ifndef MT_optimization_h
#define MT_optimization_h

#include "array.h"
#include "util.h"

//===========================================================================
//
// return types
//

/// return type for a function that returns a square potential $f(x) = x^T A x - 2 a^T x + c
struct     SqrPotential { arr A, a;          double c; };
/// return type for a function that returns a square potential $f(x,y) = [x,y]^T [A,C; C^T,B] [x,y] - 2 [a,b]^T [x,y] + c$
struct PairSqrPotential { arr A, B, C, a, b; double c; };


//===========================================================================
//
// (cost) function types
//

/// A scalar function $y = f(x)$, gradient given
struct ScalarFunction { virtual double fs(arr* grad, const arr& x) = 0; };
/// A vector function $y = f(x)$, Jacobian given
/// This implies an optimization problem $\hat f(y) = y^T(x) y(x)$ of Gauss-Newton
/// type where the Hessian is approximated by J^T J
struct VectorFunction { virtual void   fv(arr& y, arr* J, const arr& x) = 0; };

struct QuadraticFunction { virtual double fq(SqrPotential*, const arr& x) = 0; };

/// Given a chain $x_{0:T}$ of variables, implies a cost function
/// $f(x) = \sum_{i=0}^T f_i(x_i)^T f_i(x_i) + \sum_{i=1}^T f_{ij}(x_i,x_j)^T f_{ij}(x_i,x_j)$
/// and we can access local Jacobians of f_i and f_{ij}
struct VectorChainFunction {
  uint T;
  virtual void fvi (arr& y, arr* J, uint i, const arr& x_i) = 0;
  virtual void fvij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

/// Given a chain $x_{0:T}$ of variables, implies a cost function
/// $f(x) = \sum_{i=0}^T f_i(x_i) + \sum_{i=1}^T f_{ij}(x_i,x_j)$
/// and we can access local SqrPotential approximations of f_i and f_{ij}
struct SqrChainFunction {
  uint T;
  virtual double fqi (SqrPotential *S, uint i, const arr& x_i) = 0;
  virtual double fqij(PairSqrPotential *S, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

/*
struct ScalarGraphFunction {
  virtual uintA edges() = 0;
  virtual double fi (arr* grad, uint i, const arr& x_i) = 0;
  virtual double fij(arr* gradi, arr* gradj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
  double f_total(const arr& X);
};*/


//===========================================================================
//
// checks, evaluation and converters
//

void checkGradient(ScalarFunction &f, const arr& x, double tolerance);
void checkGradient(VectorFunction &f, const arr& x, double tolerance);
void checkGradient(QuadraticFunction &f, const arr& x, double tolerance);

double evaluateSP(SqrPotential& S, const arr& x);
double evaluateSF(ScalarFunction& f, const arr& x);
double evaluateVF(VectorFunction& f, const arr& x);
double evaluateVCF(VectorChainFunction& f, const arr& x);
double evaluateQCF(SqrChainFunction& f, const arr& x);

struct conv_VectorChainFunction:ScalarFunction,VectorFunction,SqrChainFunction{
  VectorChainFunction *f;
  conv_VectorChainFunction(VectorChainFunction& _f);
  virtual double fs(arr* grad, const arr& x);                  //to a ScalarFunction
  virtual void   fv(arr& y, arr* J, const arr& x);             //to a VectorFunction
  virtual double fqi (SqrPotential *S, uint i, const arr& x_i); //to a SqrChainFunction
  virtual double fqij(PairSqrPotential *S, uint i, uint j, const arr& x_i, const arr& x_j);
};


//===========================================================================
//
// optimization methods
//

/// minimizes f(x) = phi(x)^T phi(x) using the Jacobian of phi
uint optGaussNewton(arr& x, VectorFunction& phi, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, double maxStepSize=-1., uint verbose=0);

/// minimizes f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)
uint optNewton(arr& x, QuadraticFunction& f, double *fx_user=NULL, SqrPotential *S_user=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, double maxStepSize=1e-2, uint verbose=0);

/// minimizes f(x)
uint optRprop(arr& x, ScalarFunction& f, double initialStepSize, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, uint verbose=0 );

/// minimizes f(x)
uint optGradDescent(arr& x, ScalarFunction& f, double initialStepSize, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, double maxStepSize=-1., uint verbose=0 );

uint optDynamicProgramming(arr& x, SqrChainFunction& f, double *fmin_return=NULL, double stoppingTolerance=1e-2, double initialDamping=1., uint maxEvals=1000, double maxStepSize=-1., uint verbose=0 );
uint optRicatti(arr& x, SqrChainFunction& f, const arr& x0, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, double maxStepSize=-1., uint verbose=0 );
uint optNodewise(arr& x, VectorChainFunction& f, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, double maxStepSize=-1., uint verbose=0 );
uint optMinSumGaussNewton(arr& x, SqrChainFunction& f, double *fmin_return=NULL, double stoppingTolerance=1e-2, double initialDamping=1., uint maxIter=1000, double maxStepSize=-1., uint verbose=0);


//===========================================================================
//
// Rprop
//

/*! Rprop, a fast gradient-based minimization */
class Rprop {
public:
  double incr;
  double decr;
  double dMax;
  double dMin;
  double rMax;
  double delta0;
  
  arr lastGrad; // last error gradient
  arr stepSize; // last update
  
  Rprop();
  
  void init(double _delta0);
  void step(arr& x, const arr& grad, uint *singleI=NULL);
  void step(double& x, const double& grad);
  void step(arr& x, ScalarFunction& f);
  uint loop(arr& x, ScalarFunction& f, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxIterations=1000, uint verbose=0);
  bool done();
};



/*struct Monotonizer{
  enum { LevenbergMarquard=0, StepSize };
  int mode;
  double lambda;

  Monotonizer(){ mode = LevenbergMarquard; lambda=1.; }

  double check(double f_new, double f_old, double& lambda){
    if(cost>cost_old){
      damping *= 10.;
      dampingReference=b_old;
      //cout <<" AICOd REJECT: cost=" <<cost <<" cost_old=" <<cost_old <<endl;
      b = b_old;
      q = q_old;
      qhat = qhat_old;
      cost = cost_old;
      s=s_old; Sinv=Sinv_old; v=v_old; Vinv=Vinv_old; r=r_old; R=R_old;
    }else{
      damping /= 5.;
      dampingReference=b;
      //cout <<" AICOd ACCEPT" <<endl;
    }
  }
  };*/


struct OptimizationProblem {
  bool isVectorValued;
  uint N;
  arr x;
  //virtual void model(arr& output, const arr& input, const arr& x, BinaryBPNet& bp){NIY;}
  virtual double loss(const arr& x, uint i, arr *grad, double *err){NIY;} //!< loss and gradient for i-th datum and parameters x
  virtual double totalLoss(const arr& x, arr *grad, double *err){NIY;} //!< loss and gradient for i-th datum and parameters x
  
  virtual double f(arr *grad, const arr& x, int i=-1){NIY;}    //!< scalar valued function
  virtual void   F(arr& F, arr *grad, const arr& x, int i=-1){NIY;} //!< vector valued function
  OptimizationProblem(){ N=0; }
};

struct DecideSign {
  double sumX, sumXX;
  uint N;
  void init(){ N=0; sumX=sumXX=0.; }
  bool step(double x);
  double mean(){ return sumX/N; }
  double sdv(){ double m=mean(); return sqrt((sumXX+2.*m*m)/N-m*m); }
  double sign(){ return MT::sign(sumX); }
};

struct SGD {
  uint t, N;
  arr w1, w2;
  OptimizationProblem *m;
  double a1, a2, l1, l2, e1, e2;
  uintA perm;
  ofstream log;
  
#define BATCH 1000
#define UP 2.
#define DOWN 0.3
  
  void init(OptimizationProblem *_m, double initialRate, uint _N, const arr& w0){
    t=0;
    m=_m;
    a1=a2=initialRate;
    a2 *= UP;
    N=_N;
    perm.setRandomPerm(N);
    w1=w2=w0;
    l1=l2=0.;
    e1=e2=0.;
    MT::open(log, "log.sgd");
  }
  
  void stepPlain(){
    arr grad;
    double err;
    l1 += m->loss(w1, perm(t%N), &grad, &err);   w1 -= a1 * grad;   e1+=err;
    log <<t
    <<" time= " <<MT::timerRead()
    <<" loss1= " <<l1/(t%BATCH+1)
    <<" err1= "  <<e1/(t%BATCH+1)
    <<" rate1= " <<a1
    <<endl;
    cout <<t
         <<" time= " <<MT::timerRead()
         <<" loss1= " <<l1/(t%BATCH+1)
         <<" err1= "  <<e1/(t%BATCH+1)
         <<" rate1= " <<a1
         <<endl;
    t++;
    if(!(t%N)) perm.setRandomPerm(N);
    if(!(t%BATCH)){
      l1=l2=0.;
      e1=e2=0.;
    }
  }
  
  void stepTwin(){
    arr grad;
    double err;
    l1 += m->loss(w1, perm(t%N), &grad, &err);   w1 -= a1 * grad;   e1+=err;
    l2 += m->loss(w2, perm(t%N), &grad, &err);   w2 -= a2 * grad;   e2+=err;
    log <<t
    <<" time= " <<MT::timerRead()
    <<" loss1= " <<l1/(t%BATCH+1) <<" loss2= " <<l2/(t%BATCH+1)
    <<" err1= "  <<e1/(t%BATCH+1) <<" err2= "  <<e2/(t%BATCH+1)
    <<" rate1= " <<a1 <<" rate2= " <<a2
    <<endl;
    cout <<t
         <<" time= " <<MT::timerRead()
         <<" loss1= " <<l1/(t%BATCH+1) <<" loss2= " <<l2/(t%BATCH+1)
         <<" err1= "  <<e1/(t%BATCH+1) <<" err2= "  <<e2/(t%BATCH+1)
         <<" rate1= " <<a1 <<" rate2= " <<a2
         <<endl;
    t++;
    if(!(t%N)) perm.setRandomPerm(N);
    if(!(t%BATCH)){
      if(l1<=l2){  a2=a1;  w2=w1;  } else     {  a1=a2;  w1=w2;  }
      a1 *= DOWN; a2 *= UP;
      l1=l2=0.;
      e1=e2=0.;
    }
  }
};


inline double ModelStaticL(const arr& w, void* p){    return ((OptimizationProblem*)p)->totalLoss(w, NULL, NULL); }
inline void   ModelStaticDL(arr& grad, const arr& w, void* p){((OptimizationProblem*)p)->totalLoss(w, &grad, NULL); }
//void   ModelStaticF (arr& out , const arr& w, void* p){ ((OptimizationProblem*)p)->f(out, w); }
// void   ModelStaticDF(arr& grad, const arr& w, void* p){ ((OptimizationProblem*)p)->df(grad, w); }

// void checkGrad_loss(OptimizationProblem& m, const arr& w, double tolerance){
//   checkGradient(ModelStaticL, ModelStaticDL, &m, w, tolerance);
// }

// void checkGrad_fvec(OptimizationProblem& m, const arr& w, double tolerance){
//   checkGradient(ModelStaticF, ModelStaticDF, &m, w, tolerance);
// }


//===========================================================================
//
// Online Rprop
//

struct OnlineRprop {
  Rprop rprop;
  uint t, N;
  arr w;
  double l, e;
  OptimizationProblem *m;
  uintA perm;
  ofstream log;
  MT::Array<DecideSign> signer;
  
  void init(OptimizationProblem *_m, double initialRate, uint _N, const arr& w0){
    rprop.init(initialRate);
    t=0;
    m=_m;
    N=_N;
    perm.setRandomPerm(N);
    w=w0;
    signer.resize(w.N);
    for(uint i=0; i<w.N; i++) signer(i).init();
    l=0.;
    e=0.;
    MT::open(log, "log.sgd");
  }
  
  void step(){
    arr grad;
    double err;
    l += m->loss(w, perm(t%N), &grad, &err);
    e += err;
    for(uint i=0; i<w.N; i++){
      if(signer(i).step(grad(i))){  //signer is certain
        grad(i) = signer(i).sign(); //hard assign the gradient to +1 or -1
        rprop.step(w, grad, &i);      //make an rprop step only in this dimension
        signer(i).init();
        //cout <<"making step in " <<i <<endl;
      } else if(signer(i).N>1000){
        grad(i) = 0.;
        rprop.step(w, grad, &i);      //make an rprop step only in this dimension
        signer(i).init();
        //cout <<"assuming 0 grad in " <<i <<endl;
      }
    }
    log <<t
    <<" time= " <<MT::timerRead()
    <<" loss= " <<l/(t%BATCH+1)
    <<" err= "  <<e/(t%BATCH+1)
    <<endl;
    cout <<t
         <<" time= " <<MT::timerRead()
         <<" loss= " <<l/(t%BATCH+1)
         <<" err= "  <<e/(t%BATCH+1)
         <<endl;
    t++;
    if(!(t%N)) perm.setRandomPerm(N);
    if(!(t%BATCH)){
      l=0.;
      e=0.;
    }
  }
};
#undef BATCH
#undef UP
#undef DOWN

#ifdef  MT_IMPLEMENTATION
#  include "optimization.cpp"
#endif

#endif
