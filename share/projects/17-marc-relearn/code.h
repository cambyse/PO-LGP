#pragma once

#include <RL/racerEnvironment.h>
#include <RL/RL.h>
#include <RL/linearPolicy.h>
#include <Optim/gradient.h>
#include <Optim/blackbox.h>
#include <Net/net.h>
#include <Net/functions.h>

//==============================================================================

void testGradients();

arr getModelPolicyParameters();

void collectData();

void createNet(Net& N);
void writeData(Net& N);

//==============================================================================
