#include <gtest/gtest.h>

#include <vector>
#include <memory> // for shared_ptr

#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../util/ColorOutput.h"

#include "../PredictiveEnvironment.h"
#include "MinimalEnvironmentExample/MinimalEnvironment.h"
#include "../Maze/Maze.h"
#include "../CheeseMaze/CheeseMaze.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using std::vector;
using std::shared_ptr;
using ColorOutput::bold;
using ColorOutput::reset_all;

// use standard typedefs
typedef AbstractAction::ptr_t          action_ptr_t;
typedef AbstractObservation::ptr_t     observation_ptr_t;
typedef AbstractReward::ptr_t          reward_ptr_t;
typedef AbstractInstance::ptr_t        instance_ptr_t;
typedef AbstractInstance::const_ptr_t  const_instance_ptr_t;
typedef double                         probability_t;

TEST(EnvironmentTest, MinimalMemoryCheck) {
    {
        // initialize environment
        typedef shared_ptr<PredictiveEnvironment> env_ptr_t;
        vector<env_ptr_t> environments;
        environments.push_back(env_ptr_t(new MinimalEnvironment()));
        environments.push_back(env_ptr_t(new Maze(0.1)));
        for(auto env : environments) {
            // get spaces
            action_ptr_t action_space;
            observation_ptr_t observation_space;
            reward_ptr_t reward_space;
            env->get_spaces(action_space, observation_space, reward_space);
            // get all actions for random selection
            std::vector<action_ptr_t> action_vector;
            for(auto a : action_space) {
                action_vector.push_back(a);
            }
            // perform random transitions
            repeat(10) {
                action_ptr_t action = util::random_select(action_vector);
                observation_ptr_t observation;
                reward_ptr_t reward;
                env->perform_transition(action, observation, reward);
                if(DEBUG_LEVEL>1) {
                    DEBUG_OUT(0,action);
                    DEBUG_OUT(0,"    " << observation);
                    DEBUG_OUT(0,"    " << reward);
                }
            }
        }
    }
    if(AbstractInstance::memory_check_request()) {
        EXPECT_EQ(0,AbstractInstance::memory_check());
    }
}

TEST(EnvironmentTest, NormalizationAndTransitions) {

    {

        // initialize environment
        typedef shared_ptr<PredictiveEnvironment> env_ptr_t;
        vector<env_ptr_t> environments;
        environments.push_back(env_ptr_t(new MinimalEnvironment()));
        environments.push_back(env_ptr_t(new Maze(0.1)));

        // test all environments
        for(auto env : environments) {

            DEBUG_OUT(2,bold()<<"----------------"<<reset_all());

            // get spaces
            action_ptr_t action_space;
            observation_ptr_t observation_space;
            reward_ptr_t reward_space;
            env->get_spaces(action_space, observation_space, reward_space);

            // get all actions for random selection
            std::vector<action_ptr_t> action_vector;
            for(auto a : action_space) {
                action_vector.push_back(a);
            }

            // check normalization requirement (for the instance created by using
            // the action, observation, and reward spaces)
            instance_ptr_t ins = DoublyLinkedInstance::create(action_space, observation_space, reward_space);
            for(action_ptr_t a : action_space) {
                DEBUG_OUT(2,"action " << a);
                probability_t prob_sum = 0;
                for(observation_ptr_t o : observation_space) {
                    DEBUG_OUT(2,"    observation " << o);
                    for(reward_ptr_t r : reward_space) {
                        double p = env->get_prediction(ins, a, o, r);
                        DEBUG_OUT(2,"        reward " << r << " --> " << p);
                        prob_sum += p;
                    }
                }
                EXPECT_EQ(1,prob_sum) << "Unnormalized probabilities for action " << a;
            }

            // perform random transitions
            repeat(100) {
                action_ptr_t action = util::random_select(action_vector);
                observation_ptr_t observation;
                reward_ptr_t reward;
                env->perform_transition(action, observation, reward);
                if(DEBUG_LEVEL>1) {
                    DEBUG_OUT(0,action);
                    DEBUG_OUT(0,"    " << observation);
                    DEBUG_OUT(0,"    " << reward);
                }
            }
        }

        DEBUG_OUT(2,bold()<<"----------------"<<reset_all());
    }

    if(AbstractInstance::memory_check_request()) {
        EXPECT_EQ(0,AbstractInstance::memory_check());
    }
}

TEST(EnvironmentTest, Minimal) {

    {

        // initialize environment
        MinimalEnvironment mini;

        // get spaces
        action_ptr_t action_space;
        observation_ptr_t observation_space;
        reward_ptr_t reward_space;
        mini.get_spaces(action_space, observation_space, reward_space);

        // get all actions for random selection
        vector<action_ptr_t> action_vector;
        for(auto a : action_space) {
            action_vector.push_back(a);
        }

        instance_ptr_t ins = DoublyLinkedInstance::create(action_space, observation_space, reward_space);
        for(action_ptr_t a : action_space) {
            probability_t prob_sum = 0;
            for(auto o : observation_space) {
                for(auto r : reward_space) {
                    prob_sum += mini.get_prediction(ins, a, o, r);
                }
            }
            EXPECT_EQ(1,prob_sum) << "Unnormalized probabilities for action " << a;
        }

        // perform random transitions
        repeat(10) {
            mini.perform_transition(util::random_select(action_vector));
            if(DEBUG_LEVEL>1) {
                mini.print_last_transition();
            }
        }
    }

    if(AbstractInstance::memory_check_request()) {
        EXPECT_EQ(0,AbstractInstance::memory_check());
    }

}

TEST(EnvironmentTest, CheeseMaze) {

    {

        // initialize environment
        CheeseMaze cheese_maze;

        // get spaces
        action_ptr_t action_space;
        observation_ptr_t observation_space;
        reward_ptr_t reward_space;
        cheese_maze.get_spaces(action_space, observation_space, reward_space);

        // get all actions for random selection
        vector<action_ptr_t> action_vector;
        for(auto a : action_space) {
            action_vector.push_back(a);
        }

        // perform random transitions
        repeat(10) {
            cheese_maze.perform_transition(util::random_select(action_vector));
        }
    }

    if(AbstractInstance::memory_check_request()) {
        EXPECT_EQ(0,AbstractInstance::memory_check());
    }

}

TEST(EnvironmentTest, Maze) {

    {

        // initialize environment
        Maze maze;

        // get list of mazes
        auto maze_list = Maze::get_maze_list();

        for(auto name : maze_list) {

            // print
            DEBUG_OUT(1,bold() << "Testing maze '" << name << "'" << reset_all());

            // set maze
            EXPECT_TRUE(maze.set_maze(name));
            EXPECT_TRUE(maze.check_maze_definition()) << "erroneous definition of maze named '" << name << "'";

            // get spaces
            action_ptr_t action_space;
            observation_ptr_t observation_space;
            reward_ptr_t reward_space;
            maze.get_spaces(action_space, observation_space, reward_space);

            // get all actions for random selection
            vector<action_ptr_t> action_vector;
            for(auto a : action_space) {
                action_vector.push_back(a);
            }

            // simple normalization check
            instance_ptr_t ins = DoublyLinkedInstance::create(action_space, observation_space, reward_space);
            for(action_ptr_t a : action_space) {
                probability_t prob_sum = 0;
                for(auto o : observation_space) {
                    for(auto r : reward_space) {
                        prob_sum += maze.get_prediction(ins, a, o, r);
                    }
                }
                EXPECT_EQ(1,prob_sum) << "Unnormalized probabilities for action " << a;
            }

            // perform some random transitions
            repeat(10) {
                action_ptr_t action = util::random_select(action_vector);
                observation_ptr_t observation;
                reward_ptr_t reward;
                maze.perform_transition(action, observation, reward);
                if(DEBUG_LEVEL>1) {
                    DEBUG_OUT(0,action);
                    DEBUG_OUT(0,"    " << observation);
                    DEBUG_OUT(0,"    " << reward);
                }
            }
        }
    }

    if(AbstractInstance::memory_check_request()) {
        EXPECT_EQ(0,AbstractInstance::memory_check());
    }

}
