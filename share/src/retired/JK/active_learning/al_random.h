#ifndef _RANDOM_CLASSIFICATOR_H_
#define _RANDOM_CLASSIFICATOR_H_

#include "al.h"

template<class S> class Sampler;
class sRandomAL;

class RandomAL : public ActiveLearner {
  private:
    sRandomAL* s;
  public:
    virtual void setTrainingsData(const mlr::Array<arr>& data, const intA& classes);
    virtual void addData(const mlr::Array<arr>& data, const int class_);
    virtual int nextSample(mlr::Array<arr>& sample) const;
    virtual int classify(const mlr::Array<arr>& data, const int set = 0) const;

    virtual ~RandomAL() {};
    RandomAL(Sampler<mlr::Array<arr> >* sampler);
};

#endif
