#ifndef _GAUSS_PROCESS_CLASSIFICATOR_H_
#define _GAUSS_PROCESS_CLASSIFICATOR_H_

#include "al.h"
#include "al_problem.h"

template<class S> class Sampler;
class sGaussianProcessAL;

class GaussianProcessAL : public ActiveLearner {
  private:
    sGaussianProcessAL* s;
  public:
    virtual void setProblem(ActiveLearningProblem& problem);
    virtual void setTrainingsData(const mlr::Array<arr>& data, const intA& classes);
    virtual void addData(const mlr::Array<arr>& data, const int class_);
    virtual int nextSample(mlr::Array<arr>& sample) const;
    virtual int classify(const mlr::Array<arr>& data, const int set = 0) const;

    virtual ~GaussianProcessAL() {};
    GaussianProcessAL(ActiveLearningProblem& prob);
};

#endif
