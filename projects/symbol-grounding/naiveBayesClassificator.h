#ifndef NAIVEBAYESCLASSIFICATOR_H_
#define NAIVEBAYESCLASSIFICATOR_H_

#include <MT/array.h>

class sNaiveBayesClassificator;

class NaiveBayesClassificator {

private:
  sNaiveBayesClassificator* s;
  
public:
  NaiveBayesClassificator();
  virtual ~NaiveBayesClassificator();

  int classify(const MT::Array<arr>& features, int set = 0) const;
  
  void setTrainingsData(const MT::Array<arr>& features, const intA& classes);
  void nextSample(MT::Array<arr>& nextSample) const;
  void addData(const MT::Array<arr>& data, const int class_);
};

#endif // end header guard 

