#ifndef _AL_PROBLEM_H_
#define _AL_PROBLEM_H_

struct ActiveLearningProblem {
  Sampler<MT::Array<arr> >* sampler;
  Oracle* oracle;
}

#endif
