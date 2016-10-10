#pragma once

#include "RL.h"

struct LinearPolicy : mlr::Policy{
  double exploration;

  LinearPolicy();

  uint getThetaDim(const mlr::Environment& E, const mlr::Filter& F) const{ return E.getActionDim()*F.getFeatureDim(); }

  arr getInitialization(const mlr::Environment& E, const mlr::Filter& F, bool randomized=true){
    return  .1 * randn(E.getActionDim(), F.getFeatureDim());
  }

  double sampleAction(arr& action, arr& dLogPAction, const arr& inputFeatures, const arr& theta);

};
