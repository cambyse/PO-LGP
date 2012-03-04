#ifndef _LOGISTICREGRESSION_H_
#define _LOGISTICREGRESSION_H_

#include "activeLearner.h"
template<class S> class Sampler;
class sLogisticRegression;

class LogisticRegression : public ActiveLearner {
  private:
    sLogisticRegression* s;
  public:
    virtual void setTrainingsData(const MT::Array<arr>& data, const intA& classes);
    virtual void addData(const MT::Array<arr>& data, const int class_);
    virtual int nextSample(MT::Array<arr>& sample) const;
    virtual int classify(const MT::Array<arr>& data, const int set = 0) const;

    virtual ~LogisticRegression() {};
    LogisticRegression(Sampler<MT::Array<arr> >* sampler);
};

#endif
