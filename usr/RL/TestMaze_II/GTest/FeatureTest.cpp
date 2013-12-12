#include <gtest/gtest.h>
#include "../Feature.h"
#include "../AbstractAction.h"
#include "../AbstractObservation.h"
#include "../AbstractReward.h"
#include <vector>

TEST(FeatureTest, SharedPtr) {

    // vector holding pointers to basis features
    std::vector<Feature::const_feature_ptr_t> basis_features;

    // typedefs
    typedef AbstractAction::ptr_t action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t reward_ptr_t;

    // Construct basis features
    basis_features.push_back(ConstFeature::create(3.5));
    basis_features.push_back(ActionFeature::create(action_ptr_t(new AbstractAction()), 0));
    basis_features.push_back(ObservationFeature::create(observation_ptr_t(new AbstractObservation()), 0));
    basis_features.push_back(RewardFeature::create(reward_ptr_t(new AbstractReward()), 0));

    // Test counters of shared pointers
    for(auto b : basis_features) {
        // one in basis_features and one in loop variable
        EXPECT_EQ(b.use_count(),2);
    }

    // Construct compound features
    AndFeature a1(*basis_features[0]);
    AndFeature a2(*basis_features[1],*basis_features[2]);
    AndFeature a3(a1,*basis_features[3]);

    // Re-test counters
    EXPECT_EQ(3, basis_features[0].use_count()) << "in vector and two AND features (a1,a3)";
    EXPECT_EQ(2, basis_features[1].use_count()) << "in vector and one AND feature (a2)";
    EXPECT_EQ(2, basis_features[2].use_count()) << "in vector and one AND feature(a2)";
    EXPECT_EQ(2, basis_features[3].use_count()) << "in vector and one AND feature(a3)";
}
