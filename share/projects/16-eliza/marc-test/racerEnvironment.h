#pragma once

#include <RL/RL.h>
#include <Algo/kalman.h>
#include <Hardware/racer/racer.h>

struct RacerEnvironment : mlr::Environment, mlr::Filter{
  uint t;
  bool display;
  double noise;
  double theta0;

  Racer R;
  Kalman K;

  arr A,a,B; //store the transition linearizations
  arr C,c,W; //store the perception linearizations

  arr features;

  RacerEnvironment();
  virtual ~RacerEnvironment() {}

  uint getStateDim() const{ return 4; }
  uint getObservationDim() const{ return 4; }
  uint getActionDim() const{ return 1; }
  uint getFeatureDim() const{ return 4; }

  void resetEnvironment();
  virtual bool transition(arr& observation, double& reward, const arr& action);

  void resetFilter();
  void updateFilter(const arr& action, const arr& observation);
  arr& getFeatures(){ return features; }
};
