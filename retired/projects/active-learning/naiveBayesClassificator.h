#ifndef NAIVEBAYESCLASSIFICATOR_H_
#define NAIVEBAYESCLASSIFICATOR_H_

#include <Core/array.h>
#include <JK/active_learning/al.h>
#include <JK/utils/sampler.h>

class sNaiveBayesClassificator;

class NaiveBayesClassificator : public ActiveLearner{

private:
  sNaiveBayesClassificator* s;
  
public:
  NaiveBayesClassificator(Sampler<mlr::Array<arr> >* sampler);
  virtual ~NaiveBayesClassificator();

  virtual int classify(const mlr::Array<arr>& features, int set = 0) const;
  virtual void setTrainingsData(const mlr::Array<arr>& features, const intA& classes);
  virtual int nextSample(mlr::Array<arr>& nextSample) const;
  virtual void addData(const mlr::Array<arr>& data, const int class_);

};

#endif // end header guard 

