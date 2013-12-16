#include <gtest/gtest.h>

#include <vector>

#include "../util.h"

#include "../PredictiveEnvironment.h"
#include "../Maze/Maze.h"
#include "MinimalEnvironmentExample/MinimalEnvironment.h"

// TEST(EnvironmentTest, Minimal) {
//     std::vector<std::shared_ptr<PredictiveEnvironment> > environments;
//     environments.push_back(std::shared_ptr<PredictiveEnvironment>(new MinimalEnvironment()));
// }

TEST(EnvironmentTest, Minimal) {
    return;
    // use standard typedefs
    typedef AbstractAction::ptr_t      action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t      reward_ptr_t;
    // initialize environment
    MinimalEnvironment mini;
    // get spaces
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    mini.get_spaces(action_space, observation_space, reward_space);
    // get all actions for random selection
    std::vector<action_ptr_t> action_vector;
    for(auto a : action_space) {
        action_vector.push_back(a);
    }
    // perform random transitions
    repeat(20) {
        mini.perform_transition(util::random_select(action_vector));
        mini.print_last_transition();
    }
}
