#include "FeatureLearner.h"

#include <environment/Environment.h>

void FeatureLearner::set_features(const Environment & environment) {
    environment.get_features(basis_features,learner_type);
}
