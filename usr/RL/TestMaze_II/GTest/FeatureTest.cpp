#include <gtest/gtest.h>

#include "../util.h"

#include "../Feature.h"
#include "../AbstractAction.h"
#include "MinimalEnvironmentExample/MinimalAction.h"
#include "../Maze/MazeAction.h"
#include "../Maze/AugmentedMazeAction.h"
#include "../AbstractObservation.h"
#include "MinimalEnvironmentExample/MinimalObservation.h"
#include "../Maze/MazeObservation.h"
#include "../AbstractReward.h"
#include "MinimalEnvironmentExample/MinimalReward.h"
#include "../ListedReward.h"

#include <vector>

using std::vector;
using util::random_select;

TEST(FeatureTest, SharedPtr) {

    // vector holding pointers to basis features
    vector<Feature::const_feature_ptr_t> basis_features;

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

TEST(FeatureTest, ComparisonAndOrdering) {

    // typedefs
    typedef AbstractAction::ptr_t action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t reward_ptr_t;
    typedef Feature::const_feature_ptr_t f_ptr_t;

    // vector holding pointers to features
    vector<f_ptr_t> feature_vector;

    // Construct a random set of features
    repeat(1000) {
        // choose feature type at random
        switch(random_select<Feature::FEATURE_TYPE>({
                        Feature::FEATURE_TYPE::ABSTRACT,
                        Feature::FEATURE_TYPE::CONST_FEATURE,
                        Feature::FEATURE_TYPE::ACTION,
                        Feature::FEATURE_TYPE::OBSERVATION,
                        Feature::FEATURE_TYPE::REWARD,
                        Feature::FEATURE_TYPE::AND
                        })) {
        case Feature::FEATURE_TYPE::ABSTRACT: {
            feature_vector.push_back(f_ptr_t(new Feature()));
            break;
        }
        case Feature::FEATURE_TYPE::CONST_FEATURE: {
            feature_vector.push_back(ConstFeature::create(drand48()));
            break;
        }
        case Feature::FEATURE_TYPE::ACTION: {
            enum class ACTION_TYPE { NONE, MINIMAL, MAZE_ACTION, AUGMENTED_MAZE_ACTION };
            switch(random_select<AbstractAction::ACTION_TYPE>({
                            AbstractAction::ACTION_TYPE::NONE,
                            AbstractAction::ACTION_TYPE::MINIMAL,
                            AbstractAction::ACTION_TYPE::MAZE_ACTION,
                            AbstractAction::ACTION_TYPE::AUGMENTED_MAZE_ACTION
                            })) {
            case AbstractAction::ACTION_TYPE::NONE:
                feature_vector.push_back(ActionFeature::create(action_ptr_t(new AbstractAction()),rand()%11-5));
                break;
            case AbstractAction::ACTION_TYPE::MINIMAL:
                feature_vector.push_back(ActionFeature::create(action_ptr_t(new MinimalAction(random_select<MinimalAction::ACTION>({
                                            MinimalAction::ACTION::STAY,
                                            MinimalAction::ACTION::CHANGE
                                            }))),rand()%11-5));
                break;
            case AbstractAction::ACTION_TYPE::MAZE_ACTION:
                feature_vector.push_back(new ActionFeature(action_ptr_t(MazeAction(random_select<MazeAction::ACTION({MazeAction::ACTION::STAY,MazeAction::ACTION::CHANGE}))),rand()%11-5));
                break;
            case AbstractAction::ACTION_TYPE::AUGMENTED_MAZE_ACTION:
                break;
            }
            break;
        }
        case Feature::FEATURE_TYPE::OBSERVATION: {
            break;
        }
        case Feature::FEATURE_TYPE::REWARD: {
            break;
        }
        case Feature::FEATURE_TYPE::AND: {
            break;
        }
        }


        //enum class OBSERVATION_TYPE { NONE, MINIMAL, MAZE_OBSERVATION };

    // basis_features.push_back(ConstFeature::create(3.5));
    // basis_features.push_back(ActionFeature::create(action_ptr_t(new AbstractAction()), 0));
    // basis_features.push_back(ObservationFeature::create(observation_ptr_t(new AbstractObservation()), 0));
    // basis_features.push_back(RewardFeature::create(reward_ptr_t(new AbstractReward()), 0));

    // // Test counters of shared pointers
    // for(auto b : basis_features) {
    //     // one in basis_features and one in loop variable
    //     EXPECT_EQ(b.use_count(),2);
    // }

    // // Construct compound features
    // AndFeature a1(*basis_features[0]);
    // AndFeature a2(*basis_features[1],*basis_features[2]);
    // AndFeature a3(a1,*basis_features[3]);

    // // Re-test counters
    // EXPECT_EQ(3, basis_features[0].use_count()) << "in vector and two AND features (a1,a3)";
    // EXPECT_EQ(2, basis_features[1].use_count()) << "in vector and one AND feature (a2)";
    // EXPECT_EQ(2, basis_features[2].use_count()) << "in vector and one AND feature(a2)";
    // EXPECT_EQ(2, basis_features[3].use_count()) << "in vector and one AND feature(a3)";
    }
}
