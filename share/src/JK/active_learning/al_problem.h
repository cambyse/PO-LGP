#ifndef _AL_PROBLEM_H_
#define _AL_PROBLEM_H_

#include <MT/array.h>

template<class S> class Sampler;
class Oracle;
template<class S> class FeatureGenerator;

struct ActiveLearningProblem {
  Sampler<MT::Array<arr> >* sampler;
  Oracle* oracle;
  FeatureGenerator<arr >* generator;
};

#endif
