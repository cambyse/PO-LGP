#include "linearPolicy.h"

LinearPolicy::LinearPolicy(){
  exploration = mlr::getParameter<double>("RL/LinearPolicy/exploration", -1.);
  shiftOffsetInterval = 0;
}


double LinearPolicy::sampleAction(arr& action, arr& dLogPAction, const arr& inputFeatures, const arr& theta, uint t){
  action = theta * inputFeatures;
  if(shiftOffsetInterval){
    if(((t/shiftOffsetInterval)%2)==1){
      arr dFeatures = zeros(inputFeatures.N);
      dFeatures.elem(0) += 1.;
      action += theta * dFeatures;
//      CHECK(dLogPAction==NoArr, "");
    }
  }
  arr delta = zeros(action.N);
  if(exploration>0.){
    delta = exploration*randn(action.N);
    action += delta;
  }
  if(dLogPAction!=NoArr){
    dLogPAction = delta * ~inputFeatures / (exploration*exploration);
  }
  return -0.5 * sumOfSqr(delta)/(exploration*exploration);
}
