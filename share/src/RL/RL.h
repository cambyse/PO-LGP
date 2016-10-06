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
  virtual arr& getFeatures() = 0;
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
  virtual double sampleAction(arr& action, arr& dLogPAction, const arr& inputFeatures, const arr& theta) = 0;
};

//==============================================================================

struct Rollout{
  Environment& env;
  Policy& pi;
  Filter& fil;

  uint horizon;    ///< the (maximum) horizon for rollouts (terminal states shorten horizon)
  double gamma;    ///< discount factor, used to compute gradients
  int fixedRandomSeed;

  /// buffers to store the detailed trace of a rollout
  arr features, actions, dLogPActions, rewards, observations;
  double totalReturn;
  uint terminalTime;
		 
  Rollout(Environment& env, Policy& pol, Filter& fil, uint horizon, double gamma=1.);

  /// perform a rollout for given policy parameters; the full trace is stored in the buffers above; returns return
  double rollout(const arr& theta, bool computeDLogActions=true);

  ScalarFunction rolloutReturn();

  arr getGradient_REINFORCE(const arr& theta, uint numSamples);  ///< perform a rollout and return the gradient w.r.t. theta
  arr getGradient_GPOMDP(const arr& theta, uint numSamples); ///< perform a rollout and return the gradient w.r.t. theta
  arr getGradient_FiniteDifference(const arr& theta, uint averagedOverNumRndSeeds=1, double eps=1e-6); ///< uses same rnd seed for each rollout!
  arr getGradient_LinearRegression(const arr& theta, uint numSamples, double eps=1e-1, bool fixRndSeed=true); ///< take $n$ samples theta~Gauss(theta,eps^2), optionally, use same rnd seed for each rollout
};

}


