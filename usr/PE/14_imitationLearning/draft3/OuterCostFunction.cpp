#include "OuterCostFunction.h"

double OuterCostFunction::paramCosts(const arr &param) {
  double costs = 0.;

  double penalty = 0.;
  switch (penaltyType) {
  case noPenalty:
    break;
  case linear:
    for (uint i=0;i<param.d0;i++) {
      if (param(i)<paramBounds(0)) {penalty += fabs(param(i)-paramBounds(0));}
      if (param(i)>paramBounds(1)) {penalty += fabs(param(i)-paramBounds(1));}
    }
    break;
  case quadratic:
    for (uint i=0;i<param.d0;i++) {
      if (param(i)<paramBounds(0)) {penalty += pow(param(i)-paramBounds(0),2);}
      if (param(i)>paramBounds(1)) {penalty += pow(param(i)-paramBounds(1),2);}
    }
    break;
  }
  costs += penalty*penaltyFactor;

  double regularization = 0.;
  switch (regularizationType) {
  case noRegularization:
    break;
  case l1:
    regularization += sumOfAbs(param);
    break;
  case l2:
    regularization += sumOfSqr(param);
    break;
  }
  costs += regularization*regularizationFactor;
  CHECK(costs>=0,"Costs have to be greater than 0!");
  return costs;
}

double SquaredDistanceOCF::eval(const Demonstration &demA, const Demonstration &demB, const arr &param) {
  return sum((demA.qTraj - demB.qTraj)%(demA.qTraj - demB.qTraj)) + paramCosts(param);
}
