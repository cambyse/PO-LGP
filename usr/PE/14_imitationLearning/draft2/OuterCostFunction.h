#ifndef OUTERCOSTFUNCTION_H
#define OUTERCOSTFUNCTION_H

#include "Demonstration.h"

struct OuterCostFunction {
  virtual double eval(const Demonstration &demA, const Demonstration &demB) = 0;
  virtual ~OuterCostFunction(){}
};

/// sum of squared distance at each time step in joint space
struct SquaredDistanceOCF: public OuterCostFunction {
//  SquaredDistanceOCF();

//  double eval(const Demonstration &demA, const Demonstration &demB);
};


//struct GaussianMixtureOCF:OuterCostFunction {


#endif // OUTERCOSTFUNCTION_H
