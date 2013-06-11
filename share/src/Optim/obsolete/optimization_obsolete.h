/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
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



#include "optimization.h"

struct OptimizationProblem {
  bool isVectorValued;
  uint N;
  arr x;
  //virtual void model(arr& output, const arr& input, const arr& x, BinaryBPNet& bp){NIY;}
  virtual double loss(const arr& x, uint i, arr *grad, double *err) {NIY;} ///< loss and gradient for i-th datum and parameters x
  virtual double totalLoss(const arr& x, arr *grad, double *err) {NIY;} ///< loss and gradient for i-th datum and parameters x
  
  virtual double f(arr *grad, const arr& x, int i=-1) {NIY;}   ///< scalar valued function
  virtual void   F(arr& F, arr *grad, const arr& x, int i=-1) {NIY;} ///< vector valued function
  OptimizationProblem() { N=0; }
};

struct DecideSign {
  double sumX, sumXX;
  uint N;
  void init() { N=0; sumX=sumXX=0.; }
  bool step(double x);
  double mean() { return sumX/N; }
  double sdv() { double m=mean(); return sqrt((sumXX+2.*m*m)/N-m*m); }
  double sign() { return MT::sign(sumX); }
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
  
  void init(OptimizationProblem *_m, double initialRate, uint _N, const arr& w0) {
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
  
  void stepPlain() {
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
    if(!(t%BATCH)) {
      l1=l2=0.;
      e1=e2=0.;
    }
  }
  
  void stepTwin() {
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
    if(!(t%BATCH)) {
      if(l1<=l2) {  a2=a1;  w2=w1;  } else     {  a1=a2;  w1=w2;  }
      a1 *= DOWN; a2 *= UP;
      l1=l2=0.;
      e1=e2=0.;
    }
  }
};


inline double ModelStaticL(const arr& w, void* p) {    return ((OptimizationProblem*)p)->totalLoss(w, NULL, NULL); }
inline void   ModelStaticDL(arr& grad, const arr& w, void* p) {((OptimizationProblem*)p)->totalLoss(w, &grad, NULL); }
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
  
  void init(OptimizationProblem *_m, double initialRate, uint _N, const arr& w0);
  void step();
};
#undef BATCH
#undef UP
#undef DOWN
