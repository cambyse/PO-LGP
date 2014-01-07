#include "FeatureLearner.h"

#include "Environment.h"

void FeatureLearner::set_spaces(const Environment & environment) {
    environment.get_spaces(action_space,observation_space,reward_space);
}

void FeatureLearner::set_spaces(const action_ptr_t & a, const observation_ptr_t & o, const reward_ptr_t & r) {
    action_space = a;
    observation_space = o;
    reward_space = r;
}

void FeatureLearner::set_features(const Environment & environment) {
    environment.get_features(basis_features,learner_type);
}
