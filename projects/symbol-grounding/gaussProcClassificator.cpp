#include "gaussProcClassificator.h"
#include "sampler.h"
#include "al_util.h"

#include <cmath>

#include <MT/gaussianProcess.h>
#include <JK/util.h>

struct sGaussianProcessAL {
  Sampler<MT::Array<arr> >* sampler;
  GaussianProcess gp;   
  GaussKernelParams* p;

  sGaussianProcessAL(Sampler<MT::Array<arr> >* sampler) : sampler(sampler) {};
};

class GaussianProcessEvaluator : public Evaluator<MT::Array<arr> > {
  public:
    GaussianProcess& gp ;
    double evaluate(MT::Array<arr>& sample);
    GaussianProcessEvaluator(GaussianProcess& gp): gp(gp){};
};

double GaussianProcessEvaluator::evaluate(MT::Array<arr>& sample) {
   arr d;
   flatten(d, sample);

   double y, sig;
   gp.evaluate(d[0], y, sig);

   arr grad;
   gp.gradient(grad, d[0]);

   //JK_DEBUG(sig);
   //JK_DEBUG(y);
   //
   JK_DEBUG(-abs(y) + norm(grad) + sig);

   return - 10000000 * abs(y) + norm(grad) * sig; // active
   //return 1; // random
}


GaussianProcessAL::GaussianProcessAL(Sampler<MT::Array<arr> >* sampler) :
  s(new sGaussianProcessAL(sampler)) {
  s->p = new GaussKernelParams();
  s->p->obsVar = 10e-6;
  s->p->widthVar = .25;

  s->gp.setGaussKernelGP(s->p, 0);
}

void GaussianProcessAL::setTrainingsData(const MT::Array<arr>& data, const intA& classes) {
  arr d;
  flatten(d, data);
  for (uint i=0; i<d.d0; i++) {
    s->gp.appendObservation(d[i],classes(i) * 2 - 1);
  }
  s->gp.recompute();
}

void GaussianProcessAL::addData(const MT::Array<arr>& data, const int class_) {
  arr d;
  flatten(d, data);
  s->gp.appendObservation(d[0], class_ * 2 - 1);  
  s->gp.recompute();
}

int GaussianProcessAL::nextSample(MT::Array<arr>& sample) const {
  srand(time(NULL));
  rejectionSampling(sample, s->sampler, new GaussianProcessEvaluator(s->gp));
  return 1;
}

int GaussianProcessAL::classify(const MT::Array<arr>& data, const int set) const {
  double y, _unused;
  arr d;
  flatten(d, data);
  s->gp.evaluate(d[0], y, _unused);
  if (y <= 0) return 0;
  else return 1;
}


