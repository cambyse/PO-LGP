#include "al_gaussian_process.h"
#include "al_problem.h"
#include "al_util.h"

#include <cmath>

#include <MT/gaussianProcess.h>
#include <MT/array.h>
#include <JK/utils/util.h>
#include <JK/utils/sampler.h>
#include <JK/utils/featureGenerator.h>

#include <biros/logging.h>

SET_LOG(algp, INFO);

struct sGaussianProcessAL {
  GaussianProcess gp;   
  GaussKernelParams* p;

  sGaussianProcessAL() {};
};

class GaussianProcessEvaluator : public Evaluator<MT::Array<arr> > {
  public:
    GaussianProcess& gp ;
    double evaluate(MT::Array<arr>& sample);
    GaussianProcessEvaluator(GaussianProcess& gp, const ActiveLearningProblem& prb): gp(gp), problem(prb) {};
    const ActiveLearningProblem problem;
};

double cummulativeApproxVariance(int i, arr& x, GaussianProcess& gp0, GaussianProcess& gp1) {
  if ( i == x.N) {
    double sig0, sig1, y;
    gp0.evaluate(x, y, sig0);
    gp1.evaluate(x, y, sig1);
    return sig0 + sig1;
  }
  else {
    double r;
    for (double j = -.5; j < .5; j += .1) {
      x(i) = j;
      r += cummulativeApproxVariance(i+1, x, gp0, gp1);
    }
    return r;
  }
}


double GaussianProcessEvaluator::evaluate(MT::Array<arr>& sample) {
  if (MT::getParameter<bool>("random_al", false)) {
    return 0.;
  }
  else if (MT::getParameter<bool>("cummulative", false)) {
    arr d, f;
    flatten(d, sample);
    problem.generator->makeFeatures(f, d);

    GaussianProcess cp1, cp0;
    cp1.copyFrom(gp); cp0.copyFrom(gp);
    cp1.appendObservation(f[0], 1); cp0.appendObservation(f[0], -1);
    cp1.recompute(); cp0.recompute();

    arr x;
    x.resize(4);

    return - cummulativeApproxVariance(0, x, cp1, cp0);
  }
  else {
    arr d, f;
    flatten(d, sample);
    problem.generator->makeFeatures(f, d);

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
}


GaussianProcessAL::GaussianProcessAL(ActiveLearningProblem& prob) :
  s(new sGaussianProcessAL) {
  this->problem = prob;
  s->p = new GaussKernelParams();
  s->p->obsVar = 10e-6;
  s->p->widthVar = 0.01;
  s->p->priorVar = 0.1;
	s->gp.mu = -0.01;

  s->gp.setGaussKernelGP(s->p, 0);
}

void GaussianProcessAL::setTrainingsData(const MT::Array<arr>& data, const intA& classes) {
  arr d, f;
  flatten(d, data);
  problem.generator->makeFeatures(f, d);
  for (uint i=0; i<f.d0; i++) {
    s->gp.appendObservation(f[i],classes(i) * 2 - 1);
  }
  s->gp.recompute();
}

void GaussianProcessAL::addData(const MT::Array<arr>& data, const int class_) {
  arr d, f;
  flatten(d, data);
  problem.generator->makeFeatures(f, d);
  DEBUG_VAR(algp, f);
  s->gp.appendObservation(f[0], class_ * 2 - 1);  
  s->gp.recompute();
}

int GaussianProcessAL::nextSample(MT::Array<arr>& sample) const {
  rejectionSampling(sample, problem.sampler, new GaussianProcessEvaluator(s->gp, problem), 10000);
	arr d, f;
	flatten(d, sample);
  problem.generator->makeFeatures(f, d);
	
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
  problem.generator->makeFeatures(f, d);
  s->gp.evaluate(f[0], y, _unused);
  if (y <= 0) return 0;
  else return 1;
}

void GaussianProcessAL::setProblem(ActiveLearningProblem& prob) { problem = prob; }
