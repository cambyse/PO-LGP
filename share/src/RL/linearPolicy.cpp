#include "linearPolicy.h"

LinearPolicy::LinearPolicy(){
  exploration = mlr::getParameter<double>("RL/LinearPolicy/exploration", -1.);
}


double LinearPolicy::sampleAction(arr& action, arr& dLogPAction, const arr& inputFeatures, const arr& theta){
//  if(theta.N==inputFeatures.N+1){
//    action = theta.col(0) + theta.sub(0,-1,1,-1) * inputFeatures;
//  }else{
    action = theta * inputFeatures;
//  }
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
