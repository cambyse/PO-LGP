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

#include "../debug.h"

using std::vector;
using util::random_select;

typedef AbstractAction::ptr_t action_ptr_t;
typedef AbstractObservation::ptr_t observation_ptr_t;
typedef AbstractReward::ptr_t reward_ptr_t;
typedef Feature::const_feature_ptr_t f_ptr_t;

int get_time_delay();
f_ptr_t get_basis_feature();
f_ptr_t get_abstract_feature();
f_ptr_t get_const_feature();
f_ptr_t get_action_feature();
f_ptr_t get_abstract_action_feature();
f_ptr_t get_minimal_action_feature();
f_ptr_t get_maze_action_feature();
f_ptr_t get_augmented_maze_action_feature();
f_ptr_t get_observation_feature();
f_ptr_t get_abstract_observation_feature();
f_ptr_t get_minimal_observation_feature();
f_ptr_t get_maze_observation_feature();
f_ptr_t get_reward_feature();
f_ptr_t get_abstract_reward_feature();
f_ptr_t get_minimal_reward_feature();
f_ptr_t get_listed_reward_feature();
f_ptr_t get_and_feature();

TEST(FeatureTest, SharedPtr) {

    // vector holding pointers to basis features
    vector<f_ptr_t> basis_features;

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

    // vector holding pointers to features
    vector<f_ptr_t> feature_vector;

    // Construct a random set of features
    repeat(1000) {
        if(drand48()<0.8) {
            // use basis feature
            feature_vector.push_back(get_basis_feature());
        } else {
            // use 'and' feature
            feature_vector.push_back(get_and_feature());
        }
    }
}

int get_time_delay() {
    return rand()%11-5;
}

f_ptr_t get_basis_feature() {
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
        return get_abstract_feature();
    }
    case Feature::FEATURE_TYPE::CONST_FEATURE: {
        return get_const_feature();
    }
    case Feature::FEATURE_TYPE::ACTION: {
        return get_action_feature();
    }
    case Feature::FEATURE_TYPE::OBSERVATION: {
        return get_observation_feature();
    }
    case Feature::FEATURE_TYPE::REWARD: {
        return get_reward_feature();
    }
    default:
        DEBUG_ERROR("Unexpected type");
        EXPECT_TRUE(false);
        return get_abstract_feature();
    }
}

f_ptr_t get_abstract_feature() {
    return f_ptr_t(new Feature());
}

f_ptr_t get_const_feature() {
    return ConstFeature::create(drand48());
}

f_ptr_t get_action_feature() {
    switch(random_select<AbstractAction::ACTION_TYPE>({
                    AbstractAction::ACTION_TYPE::NONE,
                    AbstractAction::ACTION_TYPE::MINIMAL,
                    AbstractAction::ACTION_TYPE::MAZE_ACTION,
                    AbstractAction::ACTION_TYPE::AUGMENTED_MAZE_ACTION
                    })) {
    case AbstractAction::ACTION_TYPE::NONE:
        return get_abstract_action_feature();
    case AbstractAction::ACTION_TYPE::MINIMAL:
        return get_minimal_action_feature();
    case AbstractAction::ACTION_TYPE::MAZE_ACTION:
        return get_maze_action_feature();
    case AbstractAction::ACTION_TYPE::AUGMENTED_MAZE_ACTION:
        return get_augmented_maze_action_feature();
    default:
        DEBUG_ERROR("Unexpected type");
        EXPECT_TRUE(false);
        return get_abstract_action_feature();
    }
}

f_ptr_t get_abstract_action_feature() {
    return ActionFeature::create(action_ptr_t(new AbstractAction()),get_time_delay());
}

f_ptr_t get_minimal_action_feature() {
    return ActionFeature::create(action_ptr_t(new MinimalAction(random_select<MinimalAction::ACTION>({
                            MinimalAction::ACTION::STAY,
                            MinimalAction::ACTION::CHANGE
                            }))),get_time_delay());
}

f_ptr_t get_maze_action_feature() {
    return ActionFeature::create(action_ptr_t(new MazeAction(random_select<MazeAction::ACTION>({
                            MazeAction::ACTION::UP,
                            MazeAction::ACTION::DOWN,
                            MazeAction::ACTION::LEFT,
                            MazeAction::ACTION::RIGHT,
                            MazeAction::ACTION::STAY
                            }))),get_time_delay());
}

f_ptr_t get_augmented_maze_action_feature() {
    return ActionFeature::create(action_ptr_t(new AugmentedMazeAction(random_select<AugmentedMazeAction::ACTION>({
                            AugmentedMazeAction::ACTION::UP,
                            AugmentedMazeAction::ACTION::DOWN,
                            AugmentedMazeAction::ACTION::LEFT,
                            AugmentedMazeAction::ACTION::RIGHT,
                            AugmentedMazeAction::ACTION::STAY
                            }),
                random_select<AugmentedMazeAction::TAG>({
                            AugmentedMazeAction::TAG::TAG_0,
                            AugmentedMazeAction::TAG::TAG_1,
                            AugmentedMazeAction::TAG::TAG_2
                            }))),get_time_delay());
}

f_ptr_t get_observation_feature() {
    switch(random_select<AbstractObservation::OBSERVATION_TYPE>({
                    AbstractObservation::OBSERVATION_TYPE::NONE,
                    AbstractObservation::OBSERVATION_TYPE::MINIMAL,
                    AbstractObservation::OBSERVATION_TYPE::MAZE_OBSERVATION
                    })) {
    case AbstractObservation::OBSERVATION_TYPE::NONE:
        return get_abstract_observation_feature();
    case AbstractObservation::OBSERVATION_TYPE::MINIMAL:
        return get_minimal_observation_feature();
    case AbstractObservation::OBSERVATION_TYPE::MAZE_OBSERVATION:
        return get_maze_observation_feature();
    default:
        DEBUG_ERROR("Unexpected type");
        EXPECT_TRUE(false);
        return get_abstract_observation_feature();
    }
}

f_ptr_t get_abstract_observation_feature() {
    return ObservationFeature::create(observation_ptr_t(new AbstractObservation()),get_time_delay());
}

f_ptr_t get_minimal_observation_feature() {
    return ObservationFeature::create(observation_ptr_t(new MinimalObservation(random_select<MinimalObservation::OBSERVATION>({
                            MinimalObservation::OBSERVATION::RED,
                            MinimalObservation::OBSERVATION::GREEN
                            }))),get_time_delay());
}

f_ptr_t get_maze_observation_feature() {
    int x_dim = rand()%5 + 1;
    int y_dim = rand()%5 + 1;
    int x_pos = rand()%x_dim;
    int y_pos = rand()%y_dim;
    return ObservationFeature::create(observation_ptr_t(new MazeObservation(x_dim,y_dim,x_pos,y_pos)),get_time_delay());
}

f_ptr_t get_reward_feature() {
    switch(random_select<AbstractReward::REWARD_TYPE>({
                    AbstractReward::REWARD_TYPE::NONE,
                    AbstractReward::REWARD_TYPE::MINIMAL,
                    AbstractReward::REWARD_TYPE::LISTED_REWARD
                    })) {
    case AbstractReward::REWARD_TYPE::NONE:
        return get_abstract_reward_feature();
    case AbstractReward::REWARD_TYPE::MINIMAL:
        return get_minimal_reward_feature();
    case AbstractReward::REWARD_TYPE::LISTED_REWARD:
        return get_listed_reward_feature();
    default:
        DEBUG_ERROR("Unexpected type");
        EXPECT_TRUE(false);
        return get_abstract_reward_feature();
    }
}

f_ptr_t get_abstract_reward_feature() {
    return RewardFeature::create(reward_ptr_t(new AbstractReward()),get_time_delay());
}

f_ptr_t get_minimal_reward_feature() {
    return RewardFeature::create(reward_ptr_t(new MinimalReward(random_select<MinimalReward::REWARD>({
                            MinimalReward::REWARD::NO_REWARD,
                            MinimalReward::REWARD::SOME_REWARD
                            }))),get_time_delay());
}

f_ptr_t get_listed_reward_feature() {
    int list_length = rand()%5 + 1;
    vector<AbstractReward::value_t> reward_list(list_length);
    for(unsigned int idx=0; idx<reward_list.size(); ++idx) {
        reward_list[idx] = idx;
    }
    return RewardFeature::create(reward_ptr_t(new ListedReward(reward_list,rand()%list_length)),get_time_delay());
}

f_ptr_t get_and_feature() {
    int number_of_subfeatures = rand()%5 + 1;
    f_ptr_t and_feature(get_basis_feature());
    for(int idx=1; idx<number_of_subfeatures; ++idx) {
        and_feature = AndFeature(and_feature,get_basis_feature());
    }
    return and_feature;
}
