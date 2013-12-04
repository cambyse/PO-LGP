#include <gtest/gtest.h>

#include "../util.h"
#include "../AbstractAction.h"
#include "../Maze/MazeAction.h"
#include "../Maze/AugmentedMazeAction.h"

#include "../AbstractObservation.h"
#include "../Maze/MazeObservation.h"

#include "../AbstractReward.h"
#include "../ListedReward.h"

#define DEBUG_LEVEL 0
#define DEBUG_STRING "Testing: "
#include "../debug.h"

namespace {

    TEST(AbstractIteratableSpace, AbstractAction) {

        // construct vector with one action of every type
        std::vector<std::shared_ptr<AbstractAction> > action_vector;
        action_vector.push_back(std::make_shared<MazeAction>(MazeAction::ACTION::DOWN));
        action_vector.push_back(std::make_shared<AugmentedMazeAction>(AugmentedMazeAction::ACTION::DOWN,AugmentedMazeAction::TAG::TAG_2));

        int action_type_idx = 0;
        // for all action types (represented by one specific action of each type)
        for(std::shared_ptr<const AbstractAction> action_type : action_vector) {
            DEBUG_OUT(1,"This action: " << action_type->print());
            DEBUG_OUT(1,"    Action space:");
            // go through all actions of the corresponding action space
            int match_counter = 0;
            int match_idx = -1;
            for(auto action_in_space : *action_type) {
                DEBUG_OUT(1,"        " << action_in_space->print() );
                // make sure only a single action matches (the one we use for
                // iterating its action space)
                int action_idx = 0;
                for(auto action : action_vector) {
                    if(*action_in_space==*action) {
                        ++match_counter;
                        match_idx = action_idx;
                        DEBUG_OUT(1,"        --> " << action->print() << " (match)");
                    }
                    ++action_idx;
                }
            }
            EXPECT_EQ(1,match_counter) << "expecting exactly one action from each action type/space";
            EXPECT_EQ(action_type_idx,match_idx) << "match should be the action currently used to iterate its space";
            ++action_type_idx;
        }
    }

    TEST(AbstractIteratableSpace, AbstractObservation) {

        // construct vector with one observation of every type
        std::vector<std::shared_ptr<AbstractObservation> > observation_vector;
        observation_vector.push_back(std::make_shared<MazeObservation>(10,10,3,4));

        int observation_type_idx = 0;
        // for all observation types (represented by one specific observation of each type)
        for(std::shared_ptr<const AbstractObservation> observation_type : observation_vector) {
            DEBUG_OUT(1,"This observation: " << observation_type->print());
            DEBUG_OUT(1,"    Observation space:");
            // go through all observations of the corresponding observation space
            int match_counter = 0;
            int match_idx = -1;
            for(auto observation_in_space : *observation_type) {
                DEBUG_OUT(1,"        " << observation_in_space->print() );
                // make sure only a single observation matches (the one we use for
                // iterating its observation space)
                int observation_idx = 0;
                for(auto observation : observation_vector) {
                    if(*observation_in_space==*observation) {
                        ++match_counter;
                        match_idx = observation_idx;
                        DEBUG_OUT(1,"        --> " << observation->print() << " (match)");
                    }
                    ++observation_idx;
                }
            }
            EXPECT_EQ(1,match_counter) << "expecting exactly one observation from each observation type/space";
            EXPECT_EQ(observation_type_idx,match_idx) << "match should be the observation currently used to iterate its space";
            ++observation_type_idx;
        }
    }

    TEST(AbstractIteratableSpace, AbstractReward) {

        // construct vector with one reward of every type
        std::vector<std::shared_ptr<AbstractReward> > reward_vector;
        reward_vector.push_back(std::shared_ptr<ListedReward>(new ListedReward({0,0.1,1,5,20},1)));

        int reward_type_idx = 0;
        // for all reward types (represented by one specific reward of each type)
        for(std::shared_ptr<const AbstractReward> reward_type : reward_vector) {
            DEBUG_OUT(1,"This reward: " << reward_type->print());
            DEBUG_OUT(1,"    Reward space:");
            // go through all rewards of the corresponding reward space
            int match_counter = 0;
            int match_idx = -1;
            for(auto reward_in_space : *reward_type) {
                DEBUG_OUT(1,"        " << reward_in_space->print() );
                // make sure only a single reward matches (the one we use for
                // iterating its reward space)
                int reward_idx = 0;
                for(auto reward : reward_vector) {
                    if(*reward_in_space==*reward) {
                        ++match_counter;
                        match_idx = reward_idx;
                        DEBUG_OUT(1,"        --> " << reward->print() << " (match)");
                    }
                    ++reward_idx;
                }
            }
            EXPECT_EQ(1,match_counter) << "expecting exactly one reward from each reward type/space";
            EXPECT_EQ(reward_type_idx,match_idx) << "match should be the reward currently used to iterate its space";
            ++reward_type_idx;
        }
    }

}; // end namespace
