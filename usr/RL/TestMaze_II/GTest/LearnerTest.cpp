#include <gtest/gtest.h>

#include "../util/ColorOutput.h"

#include "../Maze/Maze.h"
#include "../KMarkovCRF.h"
#include "../UTree.h"

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

    // initialize environment and learner
    Maze maze;
    KMarkovCRF crf;

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
    repeat(100) {
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
}

TEST(LearnerTest, UTree) {

    // use standard typedefs
    typedef AbstractAction::ptr_t      action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t      reward_ptr_t;

    // initialize environment and learner
    Maze maze;
    UTree utree(0.9);

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
    }
}
