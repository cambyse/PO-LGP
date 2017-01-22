#pragma once

#include<Core/array.h>

namespace mlr {

//==============================================================================

struct Environment{
  virtual ~Environment() {}
  virtual uint getObservationDim() const = 0;
  virtual uint getActionDim() const = 0;

  /// reset state to start (typically randomized; controlled via rnd.setSeed)
  virtual void resetEnvironment() = 0;
  
  /// returns false on failure/terminal
  virtual bool transition(arr& observation, double& reward, const arr& action) = 0;
};

//==============================================================================

struct Filter{
  virtual ~Filter() {}
  virtual uint getFeatureDim() const = 0;

  /// reset the filter to its start state (prior knowledge about start state)
  virtual void resetFilter() = 0;

  /// update the filter state (e.g., a Kalman step, or history window shift)
  virtual void updateFilter(const arr& action, const arr& observation) = 0;

  /// get the policy input feature given the filter state
  virtual arr getFeatures() = 0;
};

//==============================================================================

struct Policy{
  virtual ~Policy() {}
  virtual uint getThetaDim(const Environment& E, const Filter& F) const = 0;

  /// A policy knows best what is a possible initialization in an Environment
  virtual arr getInitialization(const Environment& E, const Filter& F, bool randomized=true) = 0;

  /// given parameter theta and input features, sample an action (typically randomized)
  /// if gradLog!=NoArr, gradLog = $\del log P(a|phi)/\del theta$, for all dims of a
  /// return value is log P(a|phi)
  virtual double sampleAction(arr& action, arr& dLogPAction, const arr& inputFeatures, const arr& theta, uint t) = 0;
};

//==============================================================================

struct QFunction{
  virtual ~QFunction() {}
  virtual double operator()(const arr& inputFeatures, const arr& action) = 0;
  virtual arr getMaxAction(const arr& inputFeatures) = 0;
};

//==============================================================================

struct Rollouts{
  Environment& env;
  Policy& pi;
  Filter& fil;

  uint T;    ///< the (maximum) horizon for rollouts (terminal states shorten horizon)
  double gamma;    ///< discount factor, used to compute gradients

  /// buffers to store the detailed trace of a rollout
  uint M;          ///< number of rollouts
  arr thetas, features, actions, dLogPActions, rewards, observations, returns, returnToGo;
  uintA terminalTimes;
  double avgReturn;
		 
  Rollouts(Environment& env, Policy& pol, Filter& fil, uint T, double gamma=1.);

  /// perform a rollout for given policy parameters; the full trace is stored in the buffers above; returns return
  double rollout(uint numRollouts, const arr& theta, double thetaNoise=0., int fixedRandomSeed=-1);

  ScalarFunction rolloutReturn();

  void getBatchData(struct BatchData&);

  arr getGradient_Vanilla();  ///< return the gradient w.r.t. theta
  arr getGradient_REINFORCE();  ///< return the gradient w.r.t. theta
  arr getGradient_GPOMDP(); ///< return the gradient w.r.t. theta
  arr getGradient_FiniteDifference(const arr& theta, uint averagedOverNumRndSeeds=1, double eps=1e-6); ///< uses same rnd seed for each rollout!
  arr getGradient_LinearRegression(); ///< take $n$ samples theta~Gauss(theta,eps^2), optionally, use same rnd seed for each rollout

  arr getNaturalQParams();
  arr getFisherMatrix();

  arr getQuadraticFeature(uint m, uint t, const arr& theta);
  arr getPolynomialFeatures();

};

//==============================================================================

struct BatchData{
  arr S, A, R, Sn, An, Q;

  double bellmanError(QFunction& Qfunc, double gamma, bool onData=true);
};

}



inline uint symIndex(uint i,uint j, uint n){
  CHECK(i<n && j<n,"");
  if(j<i){ uint k=i; i=j; j=k; }
  return i*n+j-(i+1)*i/2;
}
