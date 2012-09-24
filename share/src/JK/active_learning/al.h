#ifndef _ACTIVELEARNER_H_
#define _ACTIVELEARNER_H_

#include <MT/array.h>
#include "al_problem.h"

class ActiveLearner {
  public:
    virtual void setProblem(ActiveLearningProblem& problem) = 0;
    virtual void setTrainingsData(const MT::Array<arr>& data, const intA& classes) = 0;
    virtual void addData(const MT::Array<arr>& data, const int class_) = 0;
    virtual int nextSample(MT::Array<arr>& sample) const = 0;
    virtual int classify(const MT::Array<arr>& data, const int set = 0) const = 0;

    virtual ~ActiveLearner() {};

    ActiveLearningProblem problem;
};

#endif
