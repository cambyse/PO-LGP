#include <gtest/gtest.h>

#include "../Config.h"
#include "../util/ColorOutput.h"
#include "../Maze/Maze.h"
#include "../PredictiveEnvironment.h"
#include "../Planning/LookAheadSearch.h"
#include "../Learner/KMarkovCRF.h"
#include "../Learner/UTree.h"
#include "../Learner/LinearQ.h"
#include "../Learner/TemporallyExtendedModel.h"
#include "../Learner/TemporallyExtendedLinearQ.h"
#include "../Learner/ConjunctiveAdjacency.h"
#include "../Representation/DoublyLinkedInstance.h"
#include "../Planning/LookAheadPolicy.h"

#define DEBUG_LEVEL 3
#include "../util/debug.h"

using std::vector;
using std::shared_ptr;
using std::make_shared;
using std::cout;
using std::endl;
using std::make_tuple;

using ColorOutput::bold;
using ColorOutput::reset_all;

TEST(LearnerTest, KMarkovCRF) {
    USE_CONFIG_TYPEDEFS;

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
    crf.adopt_spaces(maze);
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
    // TODO: This should be done using LookAheadPolicy as in PlannerTest.cpp
    LookAheadSearch planner(0.5);
    planner.adopt_spaces(maze);

    // start at current state
    const_instance_ptr_t maze_instance = maze.get_current_instance();
    instance_ptr_t current_instance = DoublyLinkedInstance::create(
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
        current_instance = current_instance->append(action, observation_to, reward);
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
    USE_CONFIG_TYPEDEFS;

    // initialize environment and learner
    Maze maze;
    UTree utree(0.9);
    maze.set_maze("Minimal");

    // get/set spaces and features
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    maze.get_spaces(action_space,observation_space,reward_space);
    utree.adopt_spaces(maze);
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
        const_instance_ptr_t maze_instance = maze.get_current_instance();
        instance_ptr_t current_instance = DoublyLinkedInstance::create(
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
            current_instance = current_instance->append(action, observation_to, reward);
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
        planner.adopt_spaces(maze);

        // start at current state
        const_instance_ptr_t maze_instance = maze.get_current_instance();
        instance_ptr_t current_instance = DoublyLinkedInstance::create(
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
            current_instance = current_instance->append(action, observation_to, reward);
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
    USE_CONFIG_TYPEDEFS;

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
    linQ.adopt_spaces(maze);
    linQ.set_features(maze);

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
    const_instance_ptr_t maze_instance = maze.get_current_instance();
    instance_ptr_t current_instance = DoublyLinkedInstance::create(
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
        current_instance = current_instance->append(action, observation_to, reward);
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

TEST(LearnerTest, TemporallyExtendedModel) {
    USE_CONFIG_TYPEDEFS;

    // initialize environment and learner
    Maze maze;
    shared_ptr<TemporallyExtendedModel> TEM;
    shared_ptr<ConjunctiveAdjacency> N_plus;

    // use the minimal maze
    maze.set_maze("Minimal");

    // get spaces
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    maze.get_spaces(action_space,observation_space,reward_space);

    // initialize N+, set horizon extension
    N_plus = make_shared<ConjunctiveAdjacency>();
    N_plus->adopt_spaces(maze);
    N_plus->set_horizon_extension(2);
    N_plus->set_min_horizon(-2);
    N_plus->set_max_horizon(0);
    N_plus->set_combine_features(false);
    N_plus->set_t_zero_features(ConjunctiveAdjacency::T_ZERO_FEATURES::OBSERVATION_REWARD);

    // initialize TEM using N+
    TEM = make_shared<TemporallyExtendedModel>(N_plus);
    TEM->adopt_spaces(maze);
    TEM->set_l1_factor(0.001);

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
        TEM->add_action_observation_reward_tripel(action,observation_to,reward,false);
    }

    //EXPECT_TRUE(TEM->check_derivatives(10,1));

    // try to learn something
    repeat(2) {
        TEM->grow_feature_set();
        TEM->optimize_weights_LBFGS();
        TEM->shrink_feature_set();
        TEM->print_features();
    }

    TEM->set_l1_factor(0);
    TEM->optimize_weights_LBFGS();
    TEM->print_features();

    //--------------------------------------------//
    // get likelihood for some random transitions //
    //--------------------------------------------//
    {
        double log_prob = 0;
        instance_ptr_t current_instance = maze.get_current_instance();
        int N = 100;
        repeat(N) {
            action_ptr_t action = util::random_select(action_vector);
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            maze.perform_transition(action,observation_to,reward);
            double prob = TEM->get_prediction(current_instance,action,observation_to,reward);
            DEBUG_OUT(3,current_instance << " (p = " << prob << ")");
            log_prob += log(prob);
            // update
            current_instance = current_instance->append(action,observation_to,reward);
        }
        EXPECT_NEAR(log_prob,0,-N*log(0.99)); // expect probabilites around 0.99
        DEBUG_OUT(1,"Likelihood over " << N << " random transitions: " << exp(log_prob));
        current_instance->detach_reachable();
    }

    //-----------------------------//
    // do some planned transitions //
    //-----------------------------//

    // initialize environment and planner
    LookAheadPolicy planner(0.5,TEM,false,10000);

    { // for memory check

        const_instance_ptr_t maze_instance = maze.get_current_instance();
        instance_ptr_t current_instance = DoublyLinkedInstance::create(
            maze_instance->action,
            maze_instance->observation,
            maze_instance->reward
            );

        // do several planned steps
        for(int step_idx=0; step_idx<10; ++step_idx) {

            action_ptr_t action = planner.get_action(current_instance);

            // actually perform transition and print results
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            maze.perform_transition(action,observation_to,reward);
            current_instance = current_instance->append(action, observation_to, reward);

            // print nice pictures
            if(DEBUG_LEVEL>=1) {
                maze.print_transition(action, observation_to, reward);
            }
        }

        current_instance->detach_reachable();

    }

    if(AbstractInstance::memory_check_request()) {
        EXPECT_EQ(0,AbstractInstance::memory_check());
    }
}

TEST(LearnerTest, TemporallyExtendedLinearQ) {

    USE_CONFIG_TYPEDEFS;
    typedef TemporallyExtendedLinearQ TELQ;

    // initialize environment and learner
    Maze maze;
    shared_ptr<TELQ> telq;
    shared_ptr<ConjunctiveAdjacency> N_plus;

    // use the minimal maze
    maze.set_maze("Minimal");

    // get spaces
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    maze.get_spaces(action_space,observation_space,reward_space);

    // initialize N+, set horizon extension
    N_plus = make_shared<ConjunctiveAdjacency>();
    N_plus->adopt_spaces(maze);
    N_plus->set_horizon_extension(2);
    N_plus->set_min_horizon(-2);
    N_plus->set_max_horizon(-1);
    N_plus->set_combine_features(false);
    N_plus->set_t_zero_features(ConjunctiveAdjacency::T_ZERO_FEATURES::ACTION);

    // initialize TELQ using N+
    telq = make_shared<TELQ>(N_plus, 0.1);
    telq->adopt_spaces(maze);

    // get all actions for random selection
    vector<action_ptr_t> action_vector;
    for(action_ptr_t a : action_space) {
        action_vector.push_back(a);
    }

    // do some random actions to collect data
    {
        double rew_sum = 0;
        for(int step=0; step<10000; ++step) {
            // start new episode every 100 steps
            bool new_episode = step%100==0;
            new_episode = false;
            // get action
            action_ptr_t action = util::random_select(action_vector);
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            // --------- optimal actions --------//
            // const_instance_ptr_t ins = maze.get_current_instance();
            // action = telq->optimal_2x2_policy(ins);
            // ----------------------------------//
            // perform transition
            maze.perform_transition(action,observation_to,reward);
            telq->add_action_observation_reward_tripel(action,observation_to,reward,new_episode);
            //cout << maze.get_current_instance() << endl;
            rew_sum += reward->get_value();
        }
        cout << "Sum of rewards: " << rew_sum << endl;
    }

    // construct features explicitly
    {
        TELQ::f_set_t feature_set;
        for(observation_ptr_t obs_2 : observation_space) {
            for(observation_ptr_t obs_1 : observation_space) {
                for(action_ptr_t act : action_space) {
                    feature_set.insert(f_ptr_t(new AndFeature(ObservationFeature::create(obs_2,-2),
                                                              ObservationFeature::create(obs_1,-1),
                                                              ActionFeature::create(act,0))));
                }
            }
        }
        telq->set_feature_set(feature_set);
    }

    // use optimal policy
    telq->set_optimal_2x2_policy();


    // try to learn something
    telq->set_l1_factor(1e-5);
    // do {
    //     telq->optimize_weights_Bellman_residual_error();
    //     telq->print_features();
    // } while(telq->update_policy());
    telq->run_policy_iteration();
    telq->shrink_feature_set();
    telq->print_features();
    telq->set_l1_factor(0);
    telq->optimize_weights_Bellman_residual_error();

    // make some moves
    {
        instance_ptr_t ins = maze.get_current_instance();
        double rew_sum = 0;
        int counter = 0;
        repeat(50) {
            action_ptr_t act = telq->get_action(ins);
            maze.perform_transition(act);
            ins = maze.get_current_instance();
            DEBUG_OUT(0,ins << " --> " << act);
            rew_sum += ins->reward->get_value();
            ++counter;
        }
        DEBUG_OUT(1,"mean reward = " << rew_sum/counter);
    }

    //EXPECT_TRUE(telq->check_derivatives(10,1));

    // telq->set_l1_factor(0);
    // telq->optimize_weights_LBFGS();
    // telq->print_features();
}
