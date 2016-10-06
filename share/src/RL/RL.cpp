#include "RL.h"

mlr::Rollout::Rollout(mlr::Environment& env, mlr::Policy& pol, mlr::Filter& fil, uint horizon, double gamma)
  : env(env), pi(pol), fil(fil), horizon(horizon), gamma(gamma), fixedRandomSeed(-1){

}

double mlr::Rollout::rollout(const arr& theta, bool computeDLogActions){
  if(fixedRandomSeed>=0) rnd.seed(fixedRandomSeed);

  env.resetEnvironment();
  fil.resetFilter();

  uint T=horizon, dF=fil.getFeatureDim(), dA=env.getActionDim(), dO=env.getObservationDim();
  features.resize(T, dF);
  actions.resize(T,dA);
  dLogPActions.resize(T, theta.N);
  rewards.resize(T);
  observations.resize(T,dO);

  totalReturn = 0.;
  double discount=1.;

  arr feat, act, dAct, obs; //references into the arrays...

  uint t;
  for(t=0; t<T; t++) {
    bool terminal;
    // a bit awkard, but this way is faster than using operator[]
    feat.referToDim(features, t);
    act .referToDim(actions, t);
    dAct.referToDim(dLogPActions, t);
    obs .referToDim(observations, t);
    double& reward = rewards(t);

    feat = fil.getFeatures();

    pi.sampleAction(act, dAct, feat, theta);

    terminal = env.transition(obs, reward, act);

    fil.updateFilter(act, obs);

    totalReturn += discount * reward;
    discount *= gamma;

    if(terminal) break;
  }

  terminalTime = t;
  return totalReturn;
}

ScalarFunction mlr::Rollout::rolloutReturn(){
  return [this](arr& g, arr& H, const arr& x) -> double{
    CHECK(g==NoArr,"can't analytically compute gradient");
    CHECK(H==NoArr,"can't analytically compute gradient");
    return this->rollout(x);
  };
}

arr mlr::Rollout::getGradient_REINFORCE(const arr& theta, uint numSamples){
  //-- collect routouts: rewards and dLogPActions
  arr R = zeros(numSamples);
  arr dLPA = zeros(numSamples, theta.N);
  for(uint m=0; m<numSamples; m++){
    rollout(theta);
    for(uint t=0;t<terminalTime;t++){
      R(m) += rewards(t);
      dLPA[m] += dLogPActions[t];
    }
  }

  //-- compute the bias terms for each parameter
  arr nom = zeros(theta.N);
  arr den = zeros(theta.N);
  for(uint m=0; m<numSamples; m++){
    arr aux = dLPA[m] % dLPA[m];
    nom += aux*R(m);
    den += aux;
  }
  nom /= (double)numSamples;
  den /= (double)numSamples;
  arr b = nom / den;

  //-- compute the gradient
  arr grad = zeros(theta.N);
  for(uint m=0; m<numSamples; m++){
    grad += dLPA[m] % (R(m) - b);
  }
  grad /= (double)numSamples;

  totalReturn = sum(R)/(double)numSamples;

  grad.reshapeAs(theta);
  return grad;
}

arr mlr::Rollout::getGradient_GPOMDP(const arr& theta, uint numSamples){
  //-- collect routouts, we need to store EVERYTHING for each time step
  uint T = horizon;
  arr R(numSamples, T);
  arr dLPA(numSamples, T, theta.N);
  for(uint m=0; m<numSamples; m++){
    rollout(theta);
    R[m] = rewards;
    dLPA[m] = dLogPActions;
  }

  //-- for each j and episode, compute partial sum over dLogAgion
  arr dLPA_sum = dLPA;
  for(uint m=0; m<numSamples; m++){
    for(uint j=1;j<T;j++){
      dLPA_sum.refDim(m,j) += dLPA_sum.refDim(m,j-1); //this integrates
    }
  }

  //-- compute the bias terms for each parameter and each j
  arr nom = zeros(T, theta.N);
  arr den = zeros(T, theta.N);
  for(uint m=0; m<numSamples; m++){
    for(uint j=0;j<T;j++){
      arr aux = dLPA_sum.refDim(m,j) % dLPA_sum.refDim(m,j);
      nom[j] += aux*R(m,j);
      den[j] += aux;
    }
  }
  nom /= (double)numSamples;
  den /= (double)numSamples;
  arr b = nom / den;

  //-- compute the gradient
  arr grad = zeros(theta.N);
  for(uint m=0; m<numSamples; m++){
    for(uint j=1;j<T;j++){ //HACK!!! USE at least window size 1?!
      grad += dLPA_sum.refDim(m,j) % (R(m,j) - b[j]);
    }
  }
  grad /= (double)numSamples;

  totalReturn = sum(R)/(double)numSamples;

  grad.reshapeAs(theta);

  return grad;
}

arr mlr::Rollout::getGradient_FiniteDifference(const arr& theta, uint averagedOverNumRndSeeds, double eps){
  arr grad;
  grad.resizeAs(theta);
  grad=0.;
  for(uint m=0;m<averagedOverNumRndSeeds;m++){
    fixedRandomSeed=m;
    grad += finiteDifferenceGradient(rolloutReturn(), theta);
    if(averagedOverNumRndSeeds) cout <<"  FD up to seed " <<m <<" = " <<grad/double(m+1) <<endl;
  }
  grad /= double(averagedOverNumRndSeeds);
  return grad;
}

arr mlr::Rollout::getGradient_LinearRegression(const arr& theta, uint numSamples, double eps, bool fixRndSeed){
  if(fixRndSeed)
    fixedRandomSeed=1;
  else
    fixedRandomSeed=-1;

  //collect data points (theta, return)
  arr X = eps * randn(numSamples,theta.N);
  arr y = zeros(numSamples);
  for(uint m=0;m<numSamples;m++){
    arr thetaNoisy = theta;
    thetaNoisy += X[m];
    y(m) = rollout(thetaNoisy);
    if(m>theta.N){//only to display partial result:
      arr Xsub = catCol(ones(m+1),X.refRange(0,m));
      arr ysub = y.refRange(0,m);
//      cout <<"  LinReg up to #"<< m <<" = " <<inverse_SymPosDef(~Xsub*Xsub)* ~Xsub * ysub <<endl;
    }
  }
  X = catCol(ones(numSamples),X); //add bias term
  arr b = inverse_SymPosDef(~X* X)* ~X * y;
  arr grad = b.sub(1,-1); //remove bias term (=avg return)
  grad.reshapeAs(theta);
  return grad;
}
