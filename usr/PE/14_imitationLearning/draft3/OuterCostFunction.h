#ifndef OUTERCOSTFUNCTION_H
#define OUTERCOSTFUNCTION_H

#include "Demonstration.h"

enum PenaltyType {noPenalty,linear,quadratic};
enum RegularizationType {noRegularization,l1,l2};

struct OuterCostFunction {
  arr paramBounds; // contains lower and upper bound for all parameters
  double penaltyFactor = 1.;
  double regularizationFactor = 1.;
  PenaltyType penaltyType;
  RegularizationType regularizationType;
  virtual double eval(const Demonstration &demA, const Demonstration &demB, const arr &param) = 0;
  double paramCosts(const arr &param);
  virtual ~OuterCostFunction(){}
  OuterCostFunction(PenaltyType _penaltyType,RegularizationType _regularizationType):penaltyType(_penaltyType),regularizationType(_regularizationType){}
};

/// sum of squared distance at each time step in joint space
struct SquaredDistanceOCF:OuterCostFunction {
  SquaredDistanceOCF(PenaltyType _penaltyType,RegularizationType _regularizationType):OuterCostFunction(_penaltyType,_regularizationType){}
  double eval(const Demonstration &demA, const Demonstration &demB, const arr &param);
};


//struct GaussianMixtureOCF:OuterCostFunction {


#endif // OUTERCOSTFUNCTION_H
