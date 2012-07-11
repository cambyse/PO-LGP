#include "al_gaussian_process.h"
#include "al_util.h"

#include <cmath>

#include <MT/gaussianProcess.h>
#include <MT/array.h>
#include <JK/utils/util.h>
#include <JK/utils/sampler.h>

#include <biros/logging.h>

SET_LOG(algp, DEBUG);

struct sGaussianProcessAL {
  Sampler<MT::Array<arr> >* sampler;
  GaussianProcess gp;   
  GaussKernelParams* p;

  sGaussianProcessAL(Sampler<MT::Array<arr> >* sampler) : sampler(sampler) {};
};

void makeFeatures(arr& Z, const arr& X){

  //uint n=X.d0, d=X.d1;
  //Z.resize(n, 1 + d + d*(d+1)/2  + d*(d+1)*(d+2)/6);
  //uint i, j, k, l, m;
  //for(i=0; i<n; i++){
    //arr x=X[i];
    //arr z=Z[i];
    //l=0;
    //z(l++)=1.;
    //for(j=0; j<d; j++) z(l++) = x(j);
    //for(j=0; j<d; j++) for(k=0; k<=j; k++) z(l++) = x(j)*x(k);
    //for(j=0; j<d; j++) for(k=0; k<=j; k++) for(m=0; m<=k; m++) z(l++) = x(j)*x(k)*x(m);
  //}
  Z.append(X(0,6) - X(0,2));
  Z.append(X.sub(0,-1,4,5) - X.sub(0,-1,0,1));
  Z.reshape(1,3);
  //Z = X;
}

class GaussianProcessEvaluator : public Evaluator<MT::Array<arr> > {
  public:
    GaussianProcess& gp ;
    double evaluate(MT::Array<arr>& sample);
    GaussianProcessEvaluator(GaussianProcess& gp): gp(gp){};
};


double GaussianProcessEvaluator::evaluate(MT::Array<arr>& sample) {
  if (MT::getParameter<bool>("random", false)) {
    return 0.;
  }
  arr d, f;
  flatten(d, sample);
  makeFeatures(f, d);

  double y, sig;
  gp.evaluate(f[0], y, sig);

  arr grad;
  gp.gradient(grad, f[0]);

  //JK_DEBUG(sig);
  //JK_DEBUG(y);
  //

  //DEBUG_VAR(algp, -10*fabs(y));
  return -10*fabs(y) + norm(grad)*sig;
  //return 1a // random
}


GaussianProcessAL::GaussianProcessAL(Sampler<MT::Array<arr> >* sampler) :
  s(new sGaussianProcessAL(sampler)) {
  s->p = new GaussKernelParams();
  s->p->obsVar = 10e-6;
  s->p->widthVar = 0.01;
  s->p->priorVar = 0.1;
	//s->gp.mu = -1;

  s->gp.setGaussKernelGP(s->p, 0);
}

void GaussianProcessAL::setTrainingsData(const MT::Array<arr>& data, const intA& classes) {
  arr d, f;
  flatten(d, data);
  makeFeatures(f, d);
  for (uint i=0; i<f.d0; i++) {
    s->gp.appendObservation(f[i],classes(i) * 2 - 1);
  }
  s->gp.recompute();
}

void GaussianProcessAL::addData(const MT::Array<arr>& data, const int class_) {
  arr d, f;
  flatten(d, data);
  makeFeatures(f, d);
  s->gp.appendObservation(f[0], class_ * 2 - 1);  
  s->gp.recompute();
}

int GaussianProcessAL::nextSample(MT::Array<arr>& sample) const {
  rejectionSampling(sample, s->sampler, new GaussianProcessEvaluator(s->gp), 10000);
	arr d, f;
	flatten(d, sample);
  makeFeatures(f, d);
	
   double y, sig;
   s->gp.evaluate(f[0], y, sig);

   arr grad;
   s->gp.gradient(grad, f[0]);

   //JK_DEBUG(sig);
   //JK_DEBUG(y);
   //

  return 1;
}

int GaussianProcessAL::classify(const MT::Array<arr>& data, const int set) const {
  double y, _unused;
  arr d, f;
  flatten(d, data);
  makeFeatures(f, d);
  s->gp.evaluate(f[0], y, _unused);
  if (y <= 0) return 0;
  else return 1;
}


