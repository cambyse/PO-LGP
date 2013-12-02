#include <gtest/gtest.h>
#include "../Feature.h"
#include <vector>

TEST(FeatureTest, SharedPtr) {

    // vector holding pointers to basis features
    std::vector<Feature::const_feature_ptr_t> basis_features;

    // Construct basis features
    basis_features.push_back(ConstFeature::create(3.5));
    basis_features.push_back(ActionFeature::create(Feature::action_t(), 0));
    basis_features.push_back(RewardFeature::create(Feature::reward_t(), 0));
    basis_features.push_back(RelativeObservationFeature::create(0,0,0,0));
    basis_features.push_back(ObservationFeature::create(Feature::observation_t(), 0));

    // Test counters of shared pointers
    for(auto b : basis_features) {
        // one in basis_features and one in loop variable
        EXPECT_EQ(b.use_count(),2);
    }

    // Construct compound features
    AndFeature a1(*basis_features[0]);
    AndFeature a2(*basis_features[1],*basis_features[2]);
    AndFeature a3(a1,*basis_features[3]);
    AndFeature a4(a1,a3);

    // Re-test counters
    EXPECT_EQ(4, basis_features[0].use_count());
    EXPECT_EQ(2, basis_features[1].use_count());
    EXPECT_EQ(2, basis_features[2].use_count());
    EXPECT_EQ(3, basis_features[3].use_count());
    EXPECT_EQ(1, basis_features[4].use_count());
}
