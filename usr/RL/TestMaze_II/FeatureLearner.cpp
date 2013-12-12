#include "FeatureLearner.h"

#include "PredictiveEnvironment.h"

void FeatureLearner::initialize_spaces(const PredictiveEnvironment & environment) {
    environment.get_spaces(action_space,observation_space,reward_space);
}
