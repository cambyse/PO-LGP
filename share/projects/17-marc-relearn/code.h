#pragma once

#include <RL/racerEnvironment.h>
#include <RL/RL.h>
#include <RL/linearPolicy.h>
#include <Optim/gradient.h>
#include <Optim/blackbox.h>
#include <Net/net.h>
#include <Net/functions.h>

//==============================================================================

//-- standard/previous RL methods to find ok parameters for the racer (using Kalman)
void testGradients();
arr getModelPolicyParameters();
void collectData();

struct ReLearn{
  Net N;

  void createNet(int T=-1, uint errSteps=10);
  void checkNet(const arr& x=ARR());
  void trainModel();

  void layoutNet();
  void writeData(const arr& x);
};



//==============================================================================
