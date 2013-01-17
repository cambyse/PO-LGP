#include "al_gaussian_process.h"
#include "al_problem.h"
#include "al_util.h"

#include <cmath>

#include <MT/gaussianProcess.h>
#include <MT/array.h>
#include <JK/utils/util.h>
#include <JK/utils/sampler.h>
#include <JK/utils/featureGenerator.h>

//#include <biros/logging.h>


SET_LOG(algp, DEBUG);

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

//double cummulativeApproxVariance(uint i, arr& x, GaussianProcess& gp) {
  //if ( i == x.N) {
    //double sig, y;
    //sig = 0.;
    //gp.evaluate(x, y, sig);


    ////DEBUG_VAR(algp, x);
    ////DEBUG_VAR(algp, y);
    ////DEBUG_VAR(algp, sig);
    //return sig;
  //}
  //else {
    //double r;
    //for (double j = -1.; j < 1.; j += .1) {
      //if (i != 4)
        //x(i) = j;
      //else
        //x(i) = ;
      //r += cummulativeApproxVariance(i+1, x, gp);
    //}
    //return r;
  //}
  //}
double cummulativeApproxVariance(const ActiveLearningProblem &problem, GaussianProcess& gp) {
  double cum_sig = 0;
  MT::Array<arr> sample;
  std::ifstream samples("samples.data");
  for(int i=0; i<100; ++i) {
    samples >> sample;
    arr d, f;
    flatten(d, sample);
    problem.generator->makeFeatures(f, d);
    double y, sig;
    gp.evaluate(f[0], y, sig);
    cum_sig += sig;
  }

  return cum_sig;
}


double GaussianProcessEvaluator::evaluate(MT::Array<arr>& sample) {
  if (MT::getParameter<bool>("random_al", false)) {
    return rand();
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

    //return - cummulativeApproxVariance(0, x, cp0) - cummulativeApproxVariance(0, x, cp1);
    return - cummulativeApproxVariance(problem, cp0) - cummulativeApproxVariance(problem, cp1);
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
    //return -10*fabs(y) +sig;
    //return -10*fabs(y) + norm(grad);
    //return sig;
    //return 1a // random
  }
}


GaussianProcessAL::GaussianProcessAL(ActiveLearningProblem& prob) :
  s(new sGaussianProcessAL) {
  this->problem = prob;
  s->p = new GaussKernelParams();
  s->p->widthVar = 0.05;
  s->p->priorVar = 0.1;
	s->gp.mu = -1.0;
  s->gp.obsVar = 10e-6;

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
  
  //std::ofstream cum("cum.data", std::ios::app);

  //arr x;
  //x.resize(4);
  //double y, sig;
  //s->gp.evaluate(ARR(0., 0., 0., 7.), y, sig);
  //DEBUG_VAR(algp, sig);

  //cum << cummulativeApproxVariance(problem, s->gp) << endl;

}

int GaussianProcessAL::nextSample(MT::Array<arr>& sample) const {
  rejectionSampling(sample, problem.sampler, new GaussianProcessEvaluator(s->gp, problem), 10000);
	//arr d, f;
	//flatten(d, sample);
  //problem.generator->makeFeatures(f, d);
	
   //double y, sig;
   //s->gp.evaluate(f[0], y, sig);

   //arr grad;
   //s->gp.gradient(grad, f[0]);

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
