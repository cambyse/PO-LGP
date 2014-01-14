#include <gtest/gtest.h>

#include "../util/ColorOutput.h"

#include "../Maze/Maze.h"
#include "../KMarkovCRF.h"
#include "../UTree.h"
#include "../PredictiveEnvironment.h"
#include "../LookAheadSearch.h"
#include "../LinearQ.h"

#define DEBUG_LEVEL 1
#include "../debug.h"

using std::vector;
using std::shared_ptr;

using ColorOutput::bold;
using ColorOutput::reset_all;

TEST(LearnerTest, KMarkovCRF) {

    // use standard typedefs
    typedef AbstractAction::ptr_t      action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t      reward_ptr_t;
    typedef Instance                   instance_t;

    // initialize environment and learner
    Maze maze;
    KMarkovCRF crf;

    // use the minimal maze
    maze.set_maze("Minimal");

    // get/set spaces and features
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    maze.get_spaces(action_space,observation_space,reward_space);
    crf.set_spaces(maze);
    crf.set_features(maze);

    // get all actions for random selection
    vector<action_ptr_t> action_vector;
    for(action_ptr_t a : action_space) {
        action_vector.push_back(a);
    }

    // do some random actions to collect data
    repeat(500) {
        action_ptr_t action = util::random_select(action_vector);
        observation_ptr_t observation_to;
        reward_ptr_t reward;
        maze.perform_transition(action,observation_to,reward);
        crf.add_action_observation_reward_tripel(action,observation_to,reward,false);
    }

    // try to learn something
    double likelihood;
    repeat(2) {
        crf.construct_candidate_features(1);
        crf.score_candidates_by_gradient();
        crf.add_candidate_features_to_active(0);
        crf.optimize_model(0,100);
        crf.erase_zero_features();
    }
    crf.optimize_model(0.001,100);
    crf.erase_zero_features();
    crf.optimize_model(0,100,&likelihood);
    EXPECT_NEAR(1,likelihood,0.01);

    //-----------------------------//
    // do some planned transitions //
    //-----------------------------//

    // initialize planner
    LookAheadSearch planner(0.5);
    planner.set_spaces(action_space, observation_space, reward_space);

    // start at current state
    const instance_t * maze_instance = maze.get_current_instance();
    instance_t * current_instance = instance_t::create(
        maze_instance->action,
        maze_instance->observation,
        maze_instance->reward
        );

    // do several planned steps
    double reward_sum = 0;
    int steps = 16;
    repeat(steps) {
        // do planning and select action
        action_ptr_t action;
        int max_tree_size = 10000;
        planner.clear_tree();
        planner.build_tree(
            current_instance,
            crf,
            max_tree_size
            );
        action = planner.get_optimal_action();

        // actually perform the transition
        observation_ptr_t observation_to;
        reward_ptr_t reward;
        maze.perform_transition(action,observation_to,reward);
        current_instance = current_instance->append_instance(action, observation_to, reward);
        if(DEBUG_LEVEL>0) {
            maze.print_transition(action,observation_to,reward);
        }

        // update reward
        reward_sum += reward->get_value();
    }

    // check for performance
    if(reward_sum/steps<0.25) {
        DEBUG_WARNING("Bad performance (mean reward) of KMarkovCRF (" << reward_sum/steps << ")");
    } else {
        DEBUG_OUT(1,"KMarkovCRF performance (mean reward): " << reward_sum/steps);
    }
}

TEST(LearnerTest, UTree) {

    // use standard typedefs
    typedef AbstractAction::ptr_t      action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t      reward_ptr_t;
    typedef Instance                   instance_t;

    // initialize environment and learner
    Maze maze;
    UTree utree(0.9);
    maze.set_maze("Minimal");

    // get/set spaces and features
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    maze.get_spaces(action_space,observation_space,reward_space);
    utree.set_spaces(maze);
    utree.set_features(maze);

    // get all actions for random selection
    vector<action_ptr_t> action_vector;
    for(action_ptr_t a : action_space) {
        action_vector.push_back(a);
    }

    // do some random actions to collect data
    repeat(1000) {
        action_ptr_t action = util::random_select(action_vector);
        observation_ptr_t observation_to;
        reward_ptr_t reward;
        maze.perform_transition(action,observation_to,reward);
        utree.add_action_observation_reward_tripel(action,observation_to,reward,false);
    }

    //------------------------//
    // try to learn something //
    //------------------------//

    // value based UTree
    {
        DEBUG_OUT(1,bold() << "Expanding U-Tree (value based)" << reset_all());
        utree.set_expansion_type(UTree::UTILITY_EXPANSION);
        double score_threshold = 1e-5;
        double utree_score = DBL_MAX;
        while(utree_score >= score_threshold) {
            utree_score = utree.expand_leaf_node(score_threshold);
        }

        // do some optimal transition
        const instance_t * maze_instance = maze.get_current_instance();
        instance_t * current_instance = instance_t::create(
            maze_instance->action,
            maze_instance->observation,
            maze_instance->reward
            );
        double reward_sum = 0;
        int steps = 20;
        repeat(steps) {
            action_ptr_t action = utree.get_max_value_action(current_instance);
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            maze.perform_transition(action, observation_to, reward);
            current_instance = current_instance->append_instance(action, observation_to, reward);
            reward_sum += reward->get_value();
            if(DEBUG_LEVEL>0) {
                maze.print_transition(action, observation_to, reward);
            }
        }
        // check for performance
        if(reward_sum/steps<0.2) {
            DEBUG_WARNING("Bad performance (mean reward) of value based U-Tree (" << reward_sum/steps << ")");
        } else {
            DEBUG_OUT(1,"Performance (mean reward) of value based U-Tree: " << reward_sum/steps);
        }
    }
    utree.clear_tree();

    // model based UTree
    {
        DEBUG_OUT(1,bold() << "Expanding U-Tree (model based)" << reset_all());
        utree.set_expansion_type(UTree::OBSERVATION_REWARD_EXPANSION);
        double score_threshold = 1;
        double utree_score = DBL_MAX;
        while(utree_score >= score_threshold ) {
            utree_score = utree.expand_leaf_node(score_threshold);
        }

        // initialize planner
        LookAheadSearch planner(0.5);
        planner.set_spaces(action_space, observation_space, reward_space);

        // start at current state
        const instance_t * maze_instance = maze.get_current_instance();
        instance_t * current_instance = instance_t::create(
            maze_instance->action,
            maze_instance->observation,
            maze_instance->reward
            );

        // do several planned steps
        double reward_sum = 0;
        int steps = 20;
        repeat(steps) {
            // do planning and select action
            action_ptr_t action;
            int max_tree_size = 10000;
            planner.clear_tree();
            planner.build_tree(
                current_instance,
                utree,
                max_tree_size
                );
            action = planner.get_optimal_action();

            // actually perform the transition
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            maze.perform_transition(action,observation_to,reward);
            current_instance = current_instance->append_instance(action, observation_to, reward);
            if(DEBUG_LEVEL>0) {
                maze.print_transition(action,observation_to,reward);
            }

            // update reward
            reward_sum += reward->get_value();
        }

        // check for performance
        if(reward_sum/steps<0.20) {
            DEBUG_WARNING("Bad performance (mean reward) of model based U-Tree (" << reward_sum/steps << ")");
        } else {
            DEBUG_OUT(1,"Performance (mean reward) of model based U-Tree: " << reward_sum/steps);
        }
    }
}

TEST(LearnerTest, LinearQ) {

    // use standard typedefs
    typedef AbstractAction::ptr_t      action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t      reward_ptr_t;
    typedef Instance                   instance_t;

    // initialize environment and learner
    Maze maze;
    LinearQ linQ(0.5);

    // use the minimal maze
    maze.set_maze("Minimal");

    // get/set spaces and features
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    maze.get_spaces(action_space,observation_space,reward_space);
    linQ.set_spaces(maze);
    linQ.set_features(maze);

    // get all actions for random selection
    vector<action_ptr_t> action_vector;
    for(action_ptr_t a : action_space) {
        action_vector.push_back(a);
    }

    // do some random actions to collect data
    repeat(500) {
        action_ptr_t action = util::random_select(action_vector);
        observation_ptr_t observation_to;
        reward_ptr_t reward;
        maze.perform_transition(action,observation_to,reward);
        linQ.add_action_observation_reward_tripel(action,observation_to,reward,false);
    }

    // try to learn something
    {
        // first iteration with ridge
        linQ.set_optimization_type_TD_RIDGE()
            .set_regularization(1e-10)
            .add_all_candidates(1);
        linQ.optimize();
        // second with unregularized gradient
        linQ.set_optimization_type_TD_L1()
            .set_regularization(0)
            .set_maximum_iterations(100);
        linQ.construct_candidate_features(1);
        linQ.score_candidates_by_gradient();
        linQ.add_candidates_by_score(0);
        linQ.optimize();
        linQ.erase_features_by_weight(0);
        // third with L1 regularization
        linQ.construct_candidate_features(1);
        linQ.score_candidates_by_gradient();
        linQ.add_candidates_by_score(0);
        linQ.set_optimization_type_TD_L1()
            .set_regularization(0.0001)
            .optimize();
        linQ.erase_features_by_weight(0);
        // finish with ridge
        linQ.set_optimization_type_TD_RIDGE()
            .set_regularization(1e-10)
            .optimize();
    }

    // do some optimal transition
    const instance_t * maze_instance = maze.get_current_instance();
    instance_t * current_instance = instance_t::create(
        maze_instance->action,
        maze_instance->observation,
        maze_instance->reward
        );
    double reward_sum = 0;
    int steps = 20;
    repeat(steps) {
        action_ptr_t action = linQ.get_max_value_action(current_instance);
        observation_ptr_t observation_to;
        reward_ptr_t reward;
        maze.perform_transition(action, observation_to, reward);
        current_instance = current_instance->append_instance(action, observation_to, reward);
        reward_sum += reward->get_value();
        if(DEBUG_LEVEL>0) {
            maze.print_transition(action, observation_to, reward);
        }
    }
    // check for performance
    if(reward_sum/steps<0.2) {
        DEBUG_WARNING("Bad performance (mean reward) of value based Linear-Q (" << reward_sum/steps << ")");
    } else {
        DEBUG_OUT(1,"Performance (mean reward) of value based Linear-Q: " << reward_sum/steps);
    }
}
