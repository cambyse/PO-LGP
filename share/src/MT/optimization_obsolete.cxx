#include "optimization_obsolete.h"

#ifdef MT_GSL
#include <gsl/gsl_cdf.h>
bool DecideSign::step(double x){
  N++;
  sumX+=x;
  sumXX+=x*x;
  if(N<=10) return false;
  if(!sumX) return true;
  double m=sumX/N;
  double s=sqrt((sumXX-sumX*m)/(N-1));
  double t=sqrt(N)*fabs(m)/s;
  double T=gsl_cdf_tdist_Pinv(1.-1e-6, N-1); //decide with error-prob 1e-4 if sign is significant
  if(t>T) return true;
  return false;
}
#else
bool DecideSign::step(double x){ NIY; }
#endif


void OnlineRprop::init(OptimizationProblem *_m, double initialRate, uint _N, const arr& w0){
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
  
  void OnlineRprop::step(){
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
