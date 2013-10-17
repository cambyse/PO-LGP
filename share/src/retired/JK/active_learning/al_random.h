#ifndef _RANDOM_CLASSIFICATOR_H_
#define _RANDOM_CLASSIFICATOR_H_

#include "al.h"

template<class S> class Sampler;
class sRandomAL;

class RandomAL : public ActiveLearner {
  private:
    sRandomAL* s;
  public:
    virtual void setTrainingsData(const MT::Array<arr>& data, const intA& classes);
    virtual void addData(const MT::Array<arr>& data, const int class_);
    virtual int nextSample(MT::Array<arr>& sample) const;
    virtual int classify(const MT::Array<arr>& data, const int set = 0) const;

    virtual ~RandomAL() {};
    RandomAL(Sampler<MT::Array<arr> >* sampler);
};

#endif
