/*  Copyright (C) 2000, 2006  Marc Toussaint (mtoussai@inf.ed.ac.uk)
    under the terms of the GNU LGPL (http://www.gnu.org/copyleft/lesser.html)
    see the `util.h' file for a full copyright statement  */

#ifndef MT_gauss_h
#define MT_gauss_h

#include "array.h"
#include "functions.h"

typedef void (*Trans)(arr& x);

extern bool useC;

/*! Gaussian \ingroup infer1 */
struct Gaussian {
  arr c; //!< center (normal representation)
  arr C; //!< covariance (normal representation)
  arr u; //!< `center-vector' in canonical representation
  arr U; //!< precision matrix (canonical representation)
  
  bool okC; //!< flag if normal representation is available
  bool okU; //!< flag if canonical representation is available
  
  //! okC=okU=false
  Gaussian(){ setCU(false, false); }
  
  //! dimensionality
  uint N() const { CHECK(okC || okU, "non-initialized Gaussian (no C or U)"); if(okC) return c.N; else return u.N; }
  //! make canonical representation available
  void makeU() const; //make sure the canonical rep is ok
  //! make normal representation available
  void makeC() const; //make sure the normal rep is ok
  void setCU(bool okc, bool oku){ okC=okc; okU=oku; }
  
  //! copy operator
  void operator=(const Gaussian& x);
  
  void setC(const arr& cc, const arr& CC){ c=cc; C=CC; setCU(true, false); }
  void setU(const arr& uu, const arr& UU){ u=uu; U=UU; setCU(false, true); NIY; }
  void setUniform(uint n){
    if(useC){
      setDiagonal(n, 1e10);
      setCU(true, false);
    }else{
      u.resize(n); u.setZero();
      U.resize(n, n); U.setZero();
      setCU(false, true);
    }
  }
  void setDelta(const arr& e){
    c=e;
    C.setDiag(1e-9, e.N);
    //C.setDiag(0., e.N);
    
    setCU(true, false);
    makeU();
  }
  void setRandom(uint n){
    if(useC){
      c.resize(n); rndUniform(c, -1., 1., false);
      C.resize(n, n); rndUniform(C, .01, .1, false); C=C*~C;
      setCU(true, false);
    }else{
      c.resize(n); rndUniform(c, -1., 1., false);
      C.resize(n, n); rndUniform(C, 1e-2, 100., false); C=C*~C;
      setCU(true, false);
      makeU();
    }
  }
  
  void setConditional(uint n, double var2){
    arr d, dd; d.setDiag(1./var2, n); dd.setDiag(-1./var2, n);
    U.setBlockMatrix(d, dd, dd, d);
    u.resize(2*n);
    u.setZero();
    setCU(false, true);
  }
  void setConditional(uint n, double var1, double var2);
  void setConditional(const arr& f, const arr& F, const arr& Q);
  void setDiagonal(uint n, double var);
  void setMarginal(uint n, const Gaussian& m, uintA list){
#if 1
    m.makeU();
    u.resize(n); u.setZero();
    U.resize(n, n); U.setZero();
    uint i, j;
    for(i=0; i<list.N; i++) u(list(i))=m.u(i);
    for(i=0; i<list.N; i++) for(j=0; j<list.N; j++) U(list(i), list(j))=m.U(i, j);
    setCU(false, true);
#else
    m.makeC();
    setDiagonal(n, 1e10);
    uint i, j;
    for(i=0; i<list.N; i++) c(list(i))=m.c(i);
    for(i=0; i<list.N; i++) for(j=0; j<list.N; j++) C(list(i), list(j))=m.C(i, j);
    setCU(true, false);
#endif
  }
  void setMean(const arr& x){
    makeU(); makeC();
    c=x; u=U*x;
  }
  
  double getSdv(){ makeC(); return sqrt(trace(C)); }
  
  void write(std::ostream& os) const;
  void read(std::istream& os);
};
stdPipes(Gaussian);
inPipe(Gaussian);

typedef MT::Array<Gaussian> GaussianA;
typedef MT::Array<Gaussian*> GaussianL;

//! estimate a gaussian from a table of data points \ingroup infer1
void estimate(Gaussian &g, const arr& X);
void estimateWeighted(Gaussian& g, const arr& X, const arr& W);

/*! collapses a mixture of gaussians (or weighted gaussians: P doesn't
    have to be normalized) to a single gaussian via moment matching \ingroup infer1 */
void collapseMoG(Gaussian& g, const arr& P, const GaussianA& G);
void collapseMoG(Gaussian& g, const arr& P, const GaussianL& G, bool zeroMean=false);

/*! similar to the unscented transform: first take systematically samples
    from the Gaussian, then weight the samples proportional to f(x),
    then reestimate the Gaussian \ingroup infer1 */
void resampleAndEstimate(Gaussian& g, double(*f)(const arr& x), uint N);

//! generate a single sample from a gaussian \ingroup infer1
void sample(arr& x, const Gaussian &g);

//! generate a table of N samples from a gaussian \ingroup infer1
void sample(arr& X, uint N, const Gaussian &g);

void systematicWeightedSamples(arr& X, arr& W, const Gaussian& g);

//! check if a and b are the same gaussian [approximately - prelim] \ingroup infer1
bool sameGaussian(const Gaussian &a, const Gaussian &b, double eps=1e-8);

//! symmetric Kullback-Leibler divergence \ingroup infer1
double KLDsym(const Gaussian &a, const Gaussian &b);

/*! map an initial gaussian (a) to a new gaussian (b) via a non-linear function f
    using the unscented transform \ingroup infer1 */
void unscentedTransform(Gaussian &b, const Gaussian &a, Trans f);

//! given x~{a, A} and y|x~{Fx+f, Q}, this returns y~{c, C} \ingroup infer1
void forward(Gaussian& y, const Gaussian& x, arr& f, arr& F, arr& Q);

//! given y~{b, B} and y|x~{Fx+f, Q}, this returns x~{c, C} \ingroup infer1
void backward(Gaussian& x, const Gaussian& y, arr& f, arr& F, arr& Q, double updateStep=1.);

//! given y~{b, B} and y|x~{Fx+f, Q}, this returns x~{c, C}
//void linBwd(Gaussian& x, Gaussian& y, arr& f, arr& F);

//! multiplication: {a, A} * {b, B} = {x, X} * norm \ingroup infer1
void product(Gaussian& x, const Gaussian& a, const Gaussian& b, double *logNorm=0);

//! division: {a, A} / {b, B} = {x, X} + norm \ingroup infer1
void division(Gaussian& x, const Gaussian& a, const Gaussian& b, double *logNorm=0);

//! given x~{a, A} and y|x~{f(x), Q}, this returns y~{c, C} \ingroup infer1
void forward(Gaussian& y, const Gaussian& x, Trans f, arr& Q);

//! given x~{a, A} and y|x~{f(x), Q}, this returns y~{c, C} \ingroup infer1
//void fctFwd(Gaussian& y, Gaussian& x, Trans f);

void getLinFwdFromJoint(arr& f, arr& F, arr& Q, uint n1, uint n2, Gaussian& x);

//! given x~{a, A} and y|x~{Fx+f, Q}, this returns the joint (x, y)~{c, C} \ingroup infer1
void joinMarginalAndConditional(Gaussian& xi, Gaussian& a, arr& f, arr& F, arr& Q);

//! xi is P(x, y), returns P(y|x)={Fx+f, Q}, dx is dimensionality of x
void getConditional(const Gaussian& xi, uint cut, arr& f, arr& F, arr& Q);

//! given P(x, y), return P(y|x)*UU(x) where UU(x) is uniform
void makeConditional(Gaussian& xi, uint dx);

//! joint is P(x, y), returns P(x), dx is the dimensionality of x
void getMarginal(Gaussian& x, const Gaussian& joint, uint dx);

/*! get the marginal of (z_list(0), .., z_list(m-1)) from a joint over (z_0, .., z_(n-1)) \ingroup infer1 */
void getMarginal(Gaussian& x, const Gaussian& joint, uintA& list);

/*! -- note, this is not a proper marginal multiplication -- simply set the marginal entries (z_list(0), .., z_list(m-1)) in a joint over (z_0, .., z_(n-1)) \ingroup infer1 */
void setMarginal(Gaussian& joint, const Gaussian& marg, uintA& list);

// assumes middle split
void getMarginalsFromJoint(Gaussian& x, Gaussian& y, Gaussian& xi);

/*! given a joint (x, y)~{c, C} and a potential {b, B} for y|evidence,
    this returns the updated joint (x, y)|evidence~{d, D} \ingroup infer1 */
void multiplyToJoint(Gaussian& xi, Gaussian& b);

void blowupMarginal(Gaussian& joint, const Gaussian& x, uint dy);

double reduce(GaussianA& g, uint m, const GaussianA& f, const arr& P, bool linearInit);
double reduceIterated(GaussianA& g, uint m, const GaussianA& f, const arr& P, uint K);

void write(const GaussianA& x, const char* name);



#ifdef  MT_IMPLEMENTATION
#  include "gauss.cpp"
#endif

#endif
