#ifndef MT_optimization_h
#define MT_optimization_h

#include "array.h"
#include "util.h"

//===========================================================================
//
// problem prototypes
//

//typedef double (*ScalarFunction)(arr* optional_gradient, const arr& x, void* optional_data); 
//typedef void (*VectorFunction)(arr& output, arr* optional_Jacobian, const arr& x, void* optional_data);

struct ScalarFunction { virtual double fs(arr* grad, const arr& x) = 0; };
struct VectorFunction { virtual void   fv(arr& y, arr* J, const arr& x) = 0; };

struct     SqrPotential { arr A, a;          double hata; };
struct PairSqrPotential { arr A, B, C, a, b; double hata; };

struct VectorChainFunction {
  virtual void fi (arr& y, arr* J, uint i, const arr& x_i) = 0;
  virtual void fij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};
struct SqrChainFunction {
  virtual double fi (SqrPotential *S, uint i, const arr& x_i) = 0;
  virtual double fij(PairSqrPotential *S, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
  double f_total(const arr& x);
};

struct ConvertVector2SqrChainFunction:SqrChainFunction{
  VectorChainFunction *f;
  ConvertVector2SqrChainFunction(VectorChainFunction& _f){ f=&_f; }
  double fi (SqrPotential *S, uint i, const arr& x_i){
    arr y,J;
    f->fi(y, (S?&J:NULL), i, x_i);
    if(S){
      S->A=~J * J;
      S->a=~J * (J*x_i - y);
      S->hata=sumOfSqr(J*x_i - y);
    }
    return sumOfSqr(y);
  }
  double fij(PairSqrPotential *S, uint i, uint j, const arr& x_i, const arr& x_j){
    arr y,Ji,Jj;
    f->fij(y, (S?&Ji:NULL), (S?&Jj:NULL), i, j, x_i, x_j);
    if(S){
      S->A=~Ji*Ji;
      S->B=~Jj*Jj;
      S->C=~Ji*Jj;
      S->a=~Ji*(Ji*x_i + Jj*x_j - y);
      S->b=~Jj*(Ji*x_i + Jj*x_j - y);
      S->hata=sumOfSqr(Ji*x_i + Jj*x_j - y);
    }
    return sumOfSqr(y);
  }
};

// struct ConvertVectorChain2Function:VectorFunction{
//   VectorChainFunction *f;
//   uint T;
//   ConvertVectorChain2Function(VectorChainFunction& _f,uint _T){ f=&_f; T=_T; }
//   void   fv(arr& y, arr* J, const arr& x){
//     x.reshape(T,x.N/T);
//     arr tmp;
//     f->fi(tmp, 0, x[0]);
//     uint n=tmp.N;
//     y.resize(T+(T-1),n);
//     (*J).resize(T+(T-1),n,x.d1);
//     for(t=0;t<T;t++){
//       f->fi(y[t](), &(*J)[t], t, x[t]);
//       if(t>0)  f->fij(y[t](), &(*J)[t], t, x[t]);
// 
//   }
//   double fi (SqrPotential *S, uint i, const arr& x_i){
//     arr y,J;
//     f->fi(y, (S?&J:NULL), i, x_i);
//     if(S){
//       S->A=~J * J;
//       S->a=~J * (J*x_i - y);
//       S->hata=sumOfSqr(J*x_i - y);
//     }
//     return sumOfSqr(y);
//   }
//   double fij(PairSqrPotential *S, uint i, uint j, const arr& x_i, const arr& x_j){
//     arr y,Ji,Jj;
//     f->fij(y, (S?&Ji:NULL), (S?&Jj:NULL), i, j, x_i, x_j);
//     if(S){
//       S->A=~Ji*Ji;
//       S->B=~Jj*Jj;
//       S->C=~Ji*Jj;
//       S->a=~Ji*(Ji*x_i + Jj*x_j - y);
//       S->b=~Jj*(Ji*x_i + Jj*x_j - y);
//       S->hata=sumOfSqr(Ji*x_i + Jj*x_j - y);
//     }
//     return sumOfSqr(y);
//   }
// };
/*
struct ScalarGraphFunction {
  virtual uintA edges() = 0;
  virtual double fi (arr* grad, uint i, const arr& x_i) = 0;
  virtual double fij(arr* gradi, arr* gradj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
  double f_total(const arr& X);
};*/

//===========================================================================
//
// gradient checks
//

void checkGradient(ScalarFunction &f, const arr& x, double tolerance);
void checkGradient(VectorFunction &f, const arr& x, double tolerance);


//===========================================================================
//
// optimization methods
//

/// minimizes cost(x) = phi(x)^T phi(x) using the Jacobian of phi
uint optGaussNewton(arr& x, VectorFunction& phi, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, double maxStepSize=-1., uint verbose=0);

/// minimizes f(x)
uint optRprop(arr& x, ScalarFunction& f, double initialStepSize, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, uint verbose=0 );

/// minimizes f(x)
uint optGradDescent(arr& x, ScalarFunction& f, double initialStepSize, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, double maxStepSize=-1., uint verbose=0 );

uint optDynamicProgramming(arr& x, SqrChainFunction& f, double *fmin_return=NULL, double stoppingTolerance=1e-2, uint maxEvals=1000, double maxStepSize=-1., uint verbose=0 );


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
