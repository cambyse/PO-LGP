#ifndef _GAUSS_PROCESS_CLASSIFICATOR_H_
#define _GAUSS_PROCESS_CLASSIFICATOR_H_

#include "activeLearner.h"
template<class S> class Sampler;
class sGaussianProcessAL;

class GaussianProcessAL : public ActiveLearner {
  private:
    sGaussianProcessAL* s;
  public:
    virtual void setTrainingsData(const MT::Array<arr>& data, const intA& classes);
    virtual void addData(const MT::Array<arr>& data, const int class_);
    virtual int nextSample(MT::Array<arr>& sample) const;
    virtual int classify(const MT::Array<arr>& data, const int set = 0) const;

    virtual ~GaussianProcessAL() {};
    GaussianProcessAL(Sampler<MT::Array<arr> >* sampler);
};

#endif
