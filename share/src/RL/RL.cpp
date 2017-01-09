#include "RL.h"

mlr::Rollouts::Rollouts(mlr::Environment& env, mlr::Policy& pol, mlr::Filter& fil, uint horizon, double gamma)
  : env(env), pi(pol), fil(fil), T(horizon), gamma(gamma){

}

double mlr::Rollouts::rollout(uint numRollouts, const arr& theta, double thetaNoise, int fixedRandomSeed){

  M = numRollouts;

  uint dF=fil.getFeatureDim(), dA=env.getActionDim(), dO=env.getObservationDim();

  thetas = replicate(theta, M);
  if(thetaNoise) thetas += thetaNoise*randn(thetas.N);
  features.resize(M, T, dF).setZero();
  actions.resize(M, T, dA).setZero();
  dLogPActions.resize(M, T, theta.N).setZero();
  rewards.resize(M, T).setZero();
  observations.resize(M, T, dO).setZero();
  returns.resize(M).setZero();
  returnToGo.resize(M, T).setZero();
  terminalTimes.resize(M).setZero();

  for(uint m=0;m<M;m++){
    if(fixedRandomSeed>=0) rnd.seed(fixedRandomSeed);
    env.resetEnvironment(); //TODO: this should return a first observation!
    fil.resetFilter();      //TODO: this should receive a first observation!
    double totalReturn = 0.;
    double discount=1.;

    //tmp variables; references into the arrays...
    arr feat, act, dAct, obs, thet;
    double *reward;

    thet.referToDim(thetas, m);

    uint t;
    for(t=0; t<T; t++) {
      bool terminal;
      // a bit awkard, but this way is faster than using operator[]
      feat.referToDim(features, m, t);
      act .referToDim(actions, m, t);
      dAct.referToDim(dLogPActions, m, t);
      obs .referToDim(observations, m, t);
      reward = &rewards(m, t);

      feat = fil.getFeatures(); //h_t

      pi.sampleAction(act, dAct, feat, thet, t); //a_t

      terminal = env.transition(obs, *reward, act); //(r_t, y_{t+1}) TODO: time indexing of y is inconsistent!

      fil.updateFilter(act, obs);

      totalReturn += discount * *reward;
      discount *= gamma;

      if(terminal) break;
    }

    terminalTimes(m) = t;
    returns(m) = totalReturn;

    returnToGo(m, T-1) = rewards(m, T-1);
    for(t=T-1; t--;) {
      returnToGo(m,t) = gamma*returnToGo(m,t+1) + rewards(m,t);
    }
    CHECK(fabs(returnToGo(m,0) - totalReturn)<1e-10, "rewards=\n" <<rewards[m] <<'\n' <<returnToGo[m] <<'\n' <<sum(rewards[m]));

  }
  avgReturn = sum(returns)/(double)M;

  return avgReturn;
}

ScalarFunction mlr::Rollouts::rolloutReturn(){
  return [this](arr& g, arr& H, const arr& x) -> double{
    CHECK(g==NoArr,"can't analytically compute gradient");
    CHECK(H==NoArr,"can't analytically compute gradient");
    return this->rollout(1, x);
  };
}

void mlr::Rollouts::getBatchData(mlr::BatchData& D){
  D.S.clear();
  D.A.clear();
  D.R.clear();
  D.Sn.clear();
  D.An.clear();
  D.Q.clear();

  uint i=0;
  for(uint m=0;m<M;m++) for(uint t=0;t<terminalTimes(m)-1;t++){
    D.S.append(features(m,t, {}));
    D.A.append(actions(m,t, {}));
    D.R.append(rewards(m,t));
    D.Sn.append(features(m,t+1, {}));
    D.An.append(actions(m,t+1, {}));
    D.Q.append(returnToGo(m,t));
    i++;
  }
  D.S.reshape(i, features.d2);
  D.A.reshape(i, actions.d2);
  D.R.reshape(i);
  D.Sn.reshape(i, features.d2);
  D.An.reshape(i, actions.d2);
  D.Q.reshape(i);
}

arr mlr::Rollouts::getGradient_Vanilla(){
  //-- sum dLogPActions over t
  arr dLPA = sum(dLogPActions, 1);

  //-- compute the gradient
  arr grad = zeros(dLPA.d1);
  for(uint m=0; m<M; m++){
    grad += dLPA[m] * returns(m);
  }
  grad /= (double)M;

  return grad;
}

arr mlr::Rollouts::getGradient_REINFORCE(){
  //-- sum dLogPActions over t
  arr dLPA = sum(dLogPActions, 1);

  //-- compute the bias terms for each parameter
  arr nom = zeros(dLPA.d1);
  arr den = zeros(dLPA.d1);
  for(uint m=0; m<M; m++){
    arr aux = dLPA[m] % dLPA[m];
    nom += aux*returns(m);
    den += aux;
  }
  nom /= (double)M;
  den /= (double)M;
  arr b = nom / den;

  //-- compute the gradient
  arr grad = zeros(dLPA.d1);
  for(uint m=0; m<M; m++){
    grad += dLPA[m] % (returns(m) - b);
  }
  grad /= (double)M;

  return grad;
}

arr mlr::Rollouts::getGradient_GPOMDP(){
  //-- for each j and episode, compute partial sum over dLogAgion
  arr dLPA_sum = dLogPActions;
  for(uint m=0; m<M; m++){
    for(uint j=1; j<T; j++){
      dLPA_sum(m,j, {}) += dLPA_sum(m,j-1, {}); //this integrates
    }
  }

  //-- compute the bias terms for each parameter and each j
  arr nom = zeros(T, dLogPActions.d2);
  arr den = zeros(T, dLogPActions.d2);
  for(uint m=0; m<M; m++){
    for(uint j=0;j<T;j++){
      arr aux = dLPA_sum(m,j, {}) % dLPA_sum(m,j, {});
      nom[j] += aux*rewards(m,j);
      den[j] += aux;
    }
  }
  nom /= (double)M;
  den /= (double)M;
  arr b = nom / den;

  //-- compute the gradient
  arr grad = zeros(dLogPActions.d2);
  for(uint m=0; m<M; m++){
    for(uint j=1; j<T; j++){ //HACK!!! USE at least window size 1?!
      grad += dLPA_sum(m,j, {}) % (rewards(m,j) - b[j]);
    }
  }
  grad /= (double)M;

  return grad;
}

arr mlr::Rollouts::getGradient_LinearRegression(){
  arr X = thetas;
  X.reshape(M, X.N/M);
  X = catCol(ones(M), X); //add bias term
  arr b = inverse_SymPosDef(~X* X)* ~X * returns;
  arr grad = b.sub(1,-1); //remove bias term
  return grad;
}

arr mlr::Rollouts::getNaturalQParams(){
  arr X = dLogPActions;
  X.reshape(M*T,dLogPActions.d2); //every step is a different data point

  arr y = returnToGo;
  y.reshape(M*T);

  arr beta = inverse_SymPosDef(~X* X)* ~X * y; //regression
//  double mse = sumOfSqr(y - X*beta)/double(y.N);
//  cout <<"MSE = " <<mse <<endl;

  return beta;
}

arr mlr::Rollouts::getFisherMatrix(){
  arr X = dLogPActions;
  X.reshape(M*T,dLogPActions.d2); //every step is a different data point

  arr G = zeros(X.d1, X.d1);
  for(uint i=0;i<X.d0;i++) G += X[i] ^ X[i];

  G /= (double)M;

  return G;
}

arr mlr::Rollouts::getQuadraticFeature(uint m, uint t, const arr& theta){
  arr s = features(m,t, {});
  arr a;
  if(!&theta) a = actions(m,t, {});
  else a = theta*s;
  arr x = cat(ARR(1.), s, a); //state-action vector
  arr xx = x ^ x;
  arr phi(symIndex(x.N-1,x.N-1, x.N)+1);
  for(uint i=0;i<x.N;i++) for(uint j=i;j<x.N;j++) phi(symIndex(i,j, x.N)) = xx(i,j);
  return phi;
}

arr mlr::Rollouts::getPolynomialFeatures(){
  arr phi;

  for(uint m=0;m<M;m++) for(uint t=0;t<T;t++){
    arr sa = cat(ARR(1.), features(m,t, {}), actions(m,t, {})); //state-action vector
    arr _phi = sa ^ sa;
    for(uint i=0;i<phi.d0;i++) for(uint j=0;j<=i;j++) phi.append(_phi(i,j));
  }

  phi.reshape(M*T,phi.N/(M*T));
  return phi;
}

arr mlr::Rollouts::getGradient_FiniteDifference(const arr& theta, uint averagedOverNumRndSeeds, double eps){
  int rndSeed;
  ScalarFunction sf = [this,&rndSeed](arr& g, arr& H, const arr& x) -> double{
    CHECK(g==NoArr,"can't analytically compute gradient");
    CHECK(H==NoArr,"can't analytically compute gradient");
    return this->rollout(1, x, 0., rndSeed);
  };

  arr grad = zeros(theta.N);
  for(uint m=0;m<averagedOverNumRndSeeds;m++){
    rndSeed=m;
    grad += finiteDifferenceGradient(sf, theta);
    if(averagedOverNumRndSeeds) cout <<"  FD up to seed " <<m <<" = " <<grad/double(m+1) <<endl;
  }
  grad /= double(averagedOverNumRndSeeds);
  return grad;
}



double mlr::BatchData::bellmanError(mlr::QFunction& Qfunc, double gamma, bool onData){
  double B=0.;
  for(uint i=0;i<S.d0;i++){
    arr a = An[i];
    if(!onData) a = Qfunc.getMaxAction(Sn[i]);
    double e = (R(i) + gamma*Qfunc(Sn[i], a) - Qfunc(S[i], A[i]));
    B += e*e;
  }
  B/=double(S.d0);
  return B;
}
