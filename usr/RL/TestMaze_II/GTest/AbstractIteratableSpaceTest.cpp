#include <gtest/gtest.h>

#include "../util.h"
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

#define DEBUG_LEVEL 0
#include "../debug.h"

namespace {

    TEST(AbstractIteratableSpace, AbstractAction) {

        // typedef to improve readability
        typedef AbstractAction::ptr_t action_ptr_t;

        // construct vector with one action of every type (plus one abstract)
        std::vector<action_ptr_t> action_vector;
        action_vector.push_back(new AbstractAction());
        action_vector.push_back(new MinimalAction(MinimalAction::ACTION::STAY));
        action_vector.push_back(new MazeAction(MazeAction::ACTION::DOWN));
        action_vector.push_back(new AugmentedMazeAction(AugmentedMazeAction::ACTION::LEFT,AugmentedMazeAction::TAG::TAG_2));

        int action_type_idx = 0;
        // for all action types (represented by one specific action of each type)
        for(action_ptr_t action_type : action_vector) {
            DEBUG_OUT(1,"This action: " << action_type);
            DEBUG_OUT(1,"    Action space (size " << action_type->space_size() << ")...");
            // go through all actions of the corresponding action space
            int match_counter = 0;
            int match_idx = -1;
            int action_space_idx = 0;
            for(action_ptr_t action_in_space : action_type) {
                DEBUG_OUT(1,"        " << action_in_space);
                // make sure only a single action matches (the one we use for
                // iterating its action space)
                int action_idx = 0;
                for(action_ptr_t action : action_vector) {
                    if(*action_in_space==*action) {
                        ++match_counter;
                        match_idx = action_idx;
                        DEBUG_OUT(1,"        --> " << action << " (match)");
                    }
                    ++action_idx;
                    if(DEBUG_LEVEL>0 && action_idx>100) {
                        EXPECT_TRUE(false);
                        return;
                    }
                }
                ++action_space_idx;
                if(DEBUG_LEVEL>0 && action_space_idx>100) {
                    EXPECT_TRUE(false);
                    return;
                }
            }
            if(action_type==AbstractAction()) {
                EXPECT_EQ(0,match_counter) << "expecting empty space and hence zero matches for abstract type";
            } else {
                EXPECT_EQ(1,match_counter) << "expecting exactly one action from each action type/space";
                EXPECT_EQ(action_type_idx,match_idx) << "match should be the action currently used to iterate its space";
            }
            ++action_type_idx;
        }
    }

    TEST(AbstractIteratableSpace, AbstractObservation) {

        // typedef to improve readability
        typedef AbstractObservation::ptr_t observation_ptr_t;

        // construct vector with one observation of every type
        std::vector<observation_ptr_t> observation_vector;
        observation_vector.push_back(new AbstractObservation());
        observation_vector.push_back(new MinimalObservation(MinimalObservation::OBSERVATION::RED));
        observation_vector.push_back(new MazeObservation(10,10,3,4));

        int observation_type_idx = 0;
        // for all observation types (represented by one specific observation of each type)
        for(observation_ptr_t observation_type : observation_vector) {
            DEBUG_OUT(1,"This observation: " << observation_type);
            DEBUG_OUT(1,"    Observation space (size " << observation_type->space_size() << ")...");
            // go through all observations of the corresponding observation space
            int match_counter = 0;
            int match_idx = -1;
            int observation_space_idx = 0;
            for(observation_ptr_t observation_in_space : observation_type) {
                DEBUG_OUT(1,"        " << observation_in_space);
                // make sure only a single observation matches (the one we use for
                // iterating its observation space)
                int observation_idx = 0;
                for(observation_ptr_t observation : observation_vector) {
                    if(*observation_in_space==*observation) {
                        ++match_counter;
                        match_idx = observation_idx;
                        DEBUG_OUT(1,"        --> " << observation << " (match)");
                    }
                    ++observation_idx;
                    if(DEBUG_LEVEL>0 && observation_idx>100) {
                        EXPECT_TRUE(false);
                        return;
                    }
                }
                ++observation_space_idx;
                if(DEBUG_LEVEL>0 && observation_space_idx>100) {
                    EXPECT_TRUE(false);
                    return;
                }
            }
            if(observation_type==AbstractObservation()) {
                EXPECT_EQ(0,match_counter) << "expecting empty space and hence zero matches for abstract type";
            } else {
                EXPECT_EQ(1,match_counter) << "expecting exactly one observation from each observation type/space";
                EXPECT_EQ(observation_type_idx,match_idx) << "match should be the observation currently used to iterate its space";
            }
            ++observation_type_idx;
        }
    }

    TEST(AbstractIteratableSpace, AbstractReward) {

        // typedef to improve readability
        typedef AbstractReward::ptr_t reward_ptr_t;

        // construct vector with one reward of every type
        std::vector<reward_ptr_t> reward_vector;
        reward_vector.push_back(new AbstractReward());
        reward_vector.push_back(new MinimalReward(MinimalReward::REWARD::SOME_REWARD));
        reward_vector.push_back(new ListedReward({0,0.1,1,5,20},1));

        int reward_type_idx = 0;
        // for all reward types (represented by one specific reward of each type)
        for(reward_ptr_t reward_type : reward_vector) {
            DEBUG_OUT(1,"This reward: " << reward_type);
            DEBUG_OUT(1,"    Reward space (size " << reward_type->space_size() << ")...");
            // go through all rewards of the corresponding reward space
            int match_counter = 0;
            int match_idx = -1;
            int reward_space_idx = 0;
            for(reward_ptr_t reward_in_space : reward_type) {
                DEBUG_OUT(1,"        " << reward_in_space);
                // make sure only a single reward matches (the one we use for
                // iterating its reward space)
                int reward_idx = 0;
                for(reward_ptr_t reward : reward_vector) {
                    if(*reward_in_space==*reward) {
                        ++match_counter;
                        match_idx = reward_idx;
                        DEBUG_OUT(1,"        --> " << reward << " (match)");
                    }
                    ++reward_idx;
                    if(DEBUG_LEVEL>0 && reward_idx>100) {
                        EXPECT_TRUE(false);
                        return;
                    }
                }
                ++reward_space_idx;
                if(DEBUG_LEVEL>0 && reward_space_idx>100) {
                    EXPECT_TRUE(false);
                    return;
                }
            }
            if(reward_type==AbstractReward()) {
                EXPECT_EQ(0,match_counter) << "expecting empty space and hence zero matches for abstract type";
            } else {
                EXPECT_EQ(1,match_counter) << "expecting exactly one reward from each reward type/space";
                EXPECT_EQ(reward_type_idx,match_idx) << "match should be the reward currently used to iterate its space";
            }
            ++reward_type_idx;
        }
    }

}; // end namespace
