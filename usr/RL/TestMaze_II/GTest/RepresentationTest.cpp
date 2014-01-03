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

#include "RandomElements.h"

#include <vector>
#include <list>

#define DEBUG_LEVEL 0
#include "../debug.h"

using std::vector;
using std::list;

typedef AbstractAction::ptr_t action_ptr_t;
typedef AbstractObservation::ptr_t observation_ptr_t;
typedef AbstractReward::ptr_t reward_ptr_t;

static int number_of_elemnets = 1000;

namespace {

#define EQUALITY_AND_INEQUALITY(type,getter)            \
    typedef type::ptr_t ptr_t;                          \
    vector<ptr_t> type_vector;                          \
    repeat(number_of_elemnets) {                        \
        type_vector.push_back(getter());                \
    }                                                   \
    for(ptr_t a1 : type_vector) {                       \
        for(ptr_t a2 : type_vector) {                   \
            if(a1==a2) {                                \
                EXPECT_FALSE(a1!=a2);                   \
                EXPECT_FALSE(a1<a2);                    \
            } else {                                    \
                EXPECT_FALSE(a1==a2);                   \
                EXPECT_TRUE(a1<a2 || a2<a1);            \
            }                                           \
        }                                               \
    }

#define CHECK_SORTING                                                   \
    /* perform stupid sorting */                                        \
    list<ptr_t> sorted_list;                                            \
    for(ptr_t unsorted_elem : type_vector) {                            \
        auto insert_before = sorted_list.begin();                       \
        while(insert_before!=sorted_list.end()) {                       \
            if(*insert_before<unsorted_elem) {                          \
                ++insert_before;                                        \
            } else {                                                    \
                break;                                                  \
            }                                                           \
        }                                                               \
        sorted_list.insert(insert_before,unsorted_elem);                \
    }                                                                   \
    /* check sorting */                                                 \
    for(auto low_elem=sorted_list.begin(); low_elem!=sorted_list.end(); ++low_elem) { \
        for(auto high_elem=low_elem; high_elem!=sorted_list.end(); ++high_elem) { \
            EXPECT_FALSE(*high_elem<*low_elem) << *high_elem << "<" << *low_elem; \
        }                                                               \
    }

    TEST(RepresentationTest, ActionEqualityAndOrdering) {
        EQUALITY_AND_INEQUALITY(AbstractAction,get_random_action);
        CHECK_SORTING;
    }

    TEST(RepresentationTest, ObservationEqualityAndOrdering) {
        EQUALITY_AND_INEQUALITY(AbstractObservation,get_random_observation);
        CHECK_SORTING;
    }

    TEST(RepresentationTest, RewardEqualityAndOrdering) {
        EQUALITY_AND_INEQUALITY(AbstractReward,get_random_reward);
        CHECK_SORTING;
    }

    TEST(RepresentationTest, AbstractActionSpace) {

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

    TEST(RepresentationTest, AbstractObservationSpace) {

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

    TEST(RepresentationTest, AbstractRewardSpace) {

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


    TEST(RepresentationTest, MazeObservation) {
        // check equality of position and inequality of objects for different
        // maze dimensions
        MazeObservation o1(2,3,0,1);
        MazeObservation o2(3,2,0,1);
        MazeObservation o3(2,3,2);
        EXPECT_EQ(0,o1.get_x_pos());
        EXPECT_EQ(1,o1.get_y_pos());
        EXPECT_EQ(0,o2.get_x_pos());
        EXPECT_EQ(1,o2.get_y_pos());
        EXPECT_EQ(0,o3.get_x_pos());
        EXPECT_EQ(1,o3.get_y_pos());
        EXPECT_NE(o1,o2);
        EXPECT_NE(o2,o3);
        EXPECT_EQ(o1,o3);

        // check functions to create new observations with same dimensions
        MazeObservation o4 = o1.new_observation(1,1);
        MazeObservation o5 = o1.new_observation(3);
        EXPECT_EQ(1,o4.get_x_pos());
        EXPECT_EQ(1,o4.get_y_pos());
        EXPECT_EQ(1,o5.get_x_pos());
        EXPECT_EQ(1,o5.get_y_pos());
        EXPECT_EQ(o4,o5);

        // bound check
        DEBUG_WARNING("Expect three errors:");
        MazeObservation o6(2,2,2,1);
        MazeObservation o7(2,2,1,2);
        MazeObservation o8(2,2,4);
    }

    TEST(RepresentationTest, ListedReward) {
        // check equality of value and inequality of objects for different
        // reward lists
        ListedReward r1({0,1,2,3},1);
        ListedReward r2({0,2,1,3},2);
        ListedReward r3({0,1,2,3},2);
        EXPECT_EQ(1,r1.get_value());
        EXPECT_EQ(1,r2.get_value());
        EXPECT_EQ(2,r3.get_value());
        EXPECT_NE(r1,r2);
        EXPECT_NE(r1,r3);
        EXPECT_NE(r3,r2);

        // check value assignment
        ListedReward r4({0,0.1,1,2.1,3.5,4.},1);
        ListedReward r5({0,0.1,1,2.1,3.5,4.},1.);
        EXPECT_EQ(0.1,r4.get_value());
        EXPECT_EQ(1,r5.get_value());
        r4.set_value(3.5);
        EXPECT_EQ(3.5,r4.get_value());
        DEBUG_WARNING("Expect one error:");
        r4.set_value(3);
        EXPECT_EQ(3.5,r4.get_value());
    }

}; // end namespace
