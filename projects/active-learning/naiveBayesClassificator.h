#ifndef NAIVEBAYESCLASSIFICATOR_H_
#define NAIVEBAYESCLASSIFICATOR_H_

#include <MT/array.h>
#include "activeLearner.h"
#include "sampler.h"

class sNaiveBayesClassificator;

class NaiveBayesClassificator : public ActiveLearner{

private:
  sNaiveBayesClassificator* s;
  
public:
  NaiveBayesClassificator(Sampler<MT::Array<arr> >* sampler);
  virtual ~NaiveBayesClassificator();

  virtual int classify(const MT::Array<arr>& features, int set = 0) const;
  virtual void setTrainingsData(const MT::Array<arr>& features, const intA& classes);
  virtual int nextSample(MT::Array<arr>& nextSample) const;
  virtual void addData(const MT::Array<arr>& data, const int class_);

};

#endif // end header guard 

