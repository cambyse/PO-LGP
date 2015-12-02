#ifndef _LOGISTICREGRESSION_H_
#define _LOGISTICREGRESSION_H_

#include "al.h"
#include "al_problem.h"
#include "devTools/logging.h"

SET_LOG(lr, DEBUG);
template<class S> class Sampler;
class sLogisticRegression;

class LogisticRegression : public ActiveLearner {
  private:
    sLogisticRegression* s;
  public:
    virtual void setProblem(ActiveLearningProblem& prob);
    virtual void setTrainingsData(const mlr::Array<arr>& data, const intA& classes);
    virtual void addData(const mlr::Array<arr>& data, const int class_);
    virtual int nextSample(mlr::Array<arr>& sample) const;
    virtual int classify(const mlr::Array<arr>& data, const int set = 0) const;

    virtual ~LogisticRegression() {};
    LogisticRegression(ActiveLearningProblem& prob);

};

#endif
