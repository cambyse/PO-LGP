#include "FeatureLearner.h"

#include "PredictiveEnvironment.h"

void FeatureLearner::set_spaces(const PredictiveEnvironment & environment) {
    environment.get_spaces(action_space,observation_space,reward_space);
}

void FeatureLearner::set_features(const PredictiveEnvironment & environment) {
    environment.get_features(basis_features,learner_type);
}
