/*
 * BatchMaze.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: robert
 */

#include "BatchMaze.h"

#include "Data.h"
#include "Maze.h"
#include "QIteration.h"
#include "KMDPState.h"
#include "KMarkovCRF.h"
#include "util.h"
#include "MCTS.h"
#include "LookAheadTree.h"

#include <iostream>
#include <fstream>

#include <QString>

#define DEBUG_LEVEL 0
#include "debug.h"

#define LOG_COMMENT(x) DEBUG_OUT(1,x); log_file << "# " << x << std::endl;
#define LOG(x) DEBUG_OUT(1,x); log_file << x << std::endl;

typedef Data::action_t action_t;
typedef Data::state_t state_t;
typedef Data::reward_t reward_t;

BatchMaze::BatchMaze() {}

BatchMaze::~BatchMaze() {}

int BatchMaze::run(int argc, char *argv[]) {
    if(argc <= 1) {

        typedef Data::idx_t idx_t;

        Maze maze(0);
        KMDPState current_k_mdp_state;
        LookAheadTree look_ahead_tree(0.9);

        // random initial history
//        maze.set_current_state(rand()%Data::state_n);
//        for(unsigned long state_counter=0; state_counter<Data::k; ++state_counter) {
//            action_t random_action = rand()%Data::action_n;
//            state_t state;
//            reward_t reward;
//            maze.perform_transition(random_action,state,reward);
//            current_k_mdp_state.new_state(random_action,state,reward);
//        }

        // specific initial history
//        state_t state;
//        reward_t reward;
//        maze.set_current_state(8);
//        maze.perform_transition(Data::STAY,state,reward);
//        current_k_mdp_state.new_state(Data::STAY,state,reward);
//        maze.perform_transition(Data::STAY,state,reward);
//        current_k_mdp_state.new_state(Data::STAY,state,reward);

        for(int i=0; i<(idx_t)Data::k; ++i) {
            current_k_mdp_state.new_state(Data::STAY,Data::state_n-1,Data::min_reward);
        }

        look_ahead_tree.build_tree<Maze>(current_k_mdp_state, 4, maze, maze.get_prediction_ptr());
//        look_ahead_tree.print_tree(2,true);

        DEBUG_OUT(0, "Action values for state");
        DEBUG_OUT(0, "    " << current_k_mdp_state.print());

        for(Data::action_idx_t action_idx=0; action_idx<(idx_t)Data::action_n; ++action_idx) {
            action_t action = Data::action_from_idx(action_idx);
            DEBUG_OUT(0,
                    Data::action_strings[action_idx] << " --> " <<
                    look_ahead_tree.get_action_value(action)
                    );
        }

        DEBUG_OUT(0,"Best action:");
        for(int i=0; i<10; ++i) {
            DEBUG_OUT(0, Data::action_strings[look_ahead_tree.get_best_action()]);
        }

//        DEBUG_OUT(1, "No arguments, terminating...");
        return 1;
    } else {

        QString arg_1(argv[1]);
        double epsilon = 0.1;
        double discount = 0.9;

        if(arg_1=="optimal" || arg_1=="kmdp" || arg_1=="sparse") {
            //-------------------------------------//
            //    get epsilon and discount from    //
            //    command line arguments           //
            //-------------------------------------//
            if(argc > 2) {
                double tmp_epsilon;
                if(util::arg_double(argv[2],0,tmp_epsilon)) {
                    epsilon = tmp_epsilon;
                } else {
                    DEBUG_OUT(1, "Could not use second argument as epsilon");
                }
            }
            if(argc > 3) {
                double tmp_discount;
                if(util::arg_double(argv[3],0,tmp_discount)) {
                    discount = tmp_discount;
                } else {
                    DEBUG_OUT(1, "Could not use third argument as discout");
                }
            }
        }

        if(arg_1=="optimal") {

            //------------------------//
            //    Optimal Behavior    //
            //------------------------//

            QString log_file_name("log_file_optimal_");
            log_file_name.append(QString::number(epsilon));
            log_file_name.append("_");
            log_file_name.append(QString::number(discount));
            log_file_name.append("_");
            log_file_name.append(QString::number(rand()));
            log_file_name.append(".txt");
            std::ofstream log_file;
            log_file.open((const char*)log_file_name.toAscii());

            Maze maze(epsilon);
            QIteration qiteration(discount);
            KMDPState current_k_mdp_state;

            LOG_COMMENT("------------------------------------");
            LOG_COMMENT("Running episodes with optimal policy");
            LOG_COMMENT("------------------------------------");
            LOG_COMMENT("");

            LOG_COMMENT("Maze size: " << Data::maze_x_dim << "x" << Data::maze_y_dim);
            LOG_COMMENT("epsilon  = " << maze.get_epsilon() );
            LOG_COMMENT("discount = " << qiteration.get_discount() );
            LOG_COMMENT("");

            double max_change_threshold = 0.0001, max_change;
            LOG_COMMENT("Running Q-Iteration (max change threshold = " << max_change_threshold << ")");
            LOG_COMMENT("");
            unsigned long iteration_counter = 0;
            maze.initialize_predictions(qiteration);
            do {
                max_change = qiteration.iterate();
                ++iteration_counter;
                DEBUG_OUT(2,"Iteration " << iteration_counter << ": max_change = " << max_change);
            } while (max_change>max_change_threshold);

            unsigned long max_episodes = 100;
            unsigned long max_transition = 1000;
            unsigned long total_transition_counter = 0;
            double reward_sum = 0;
            LOG_COMMENT("Running " << max_episodes << " episodes of length " << max_transition << " from random starting state with optimal policy");
            LOG_COMMENT("");
            for(unsigned long episode_counter=1; episode_counter<=max_episodes; ++episode_counter) {

                maze.set_current_state(rand()%Data::state_n);
                for(unsigned long state_counter=0; state_counter<Data::k; ++state_counter) {
                    action_t random_action = rand()%Data::action_n;
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(random_action,state,reward);
                    current_k_mdp_state.new_state(random_action,state,reward);
                }

                for(unsigned long transition_counter=1; transition_counter<=max_transition; ++transition_counter) {
                    action_t action = qiteration.optimal_action(current_k_mdp_state.get_k_mdp_state());
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(action,state,reward);
                    current_k_mdp_state.new_state(action,state,reward);
                    //                DEBUG_OUT(1,"Transition " << transition_counter+1 << ": " << Data::action_strings[action] << " " << state << " " << reward );
                    reward_sum += reward;
                    ++total_transition_counter;
                    DEBUG_OUT(2, "Episode " << episode_counter << ", transition " << transition_counter << ": mean reward = " << reward_sum/total_transition_counter);
                }
            }

            DEBUG_OUT(1,"The mean reward was " << reward_sum/total_transition_counter);
            LOG( epsilon << " " << discount << " " << reward_sum/total_transition_counter );

            DEBUG_OUT(1,"");
            DEBUG_OUT(1,"DONE");
            DEBUG_OUT(1,"");

            log_file.close();

        } else if(arg_1=="kmdp") {

            //---------------------------------//
            //    Learned k-MDP transitions    //
            //---------------------------------//

            QString log_file_name("log_file_kmdp_");
            log_file_name.append(QString::number(epsilon));
            log_file_name.append("_");
            log_file_name.append(QString::number(discount));
            log_file_name.append("_");
            log_file_name.append(QString::number(rand()));
            log_file_name.append(".txt");
            std::ofstream log_file;
            log_file.open((const char*)log_file_name.toAscii());

            Maze maze(epsilon);
            QIteration qiteration(discount);
            KMarkovCRF crf;
            KMDPState current_k_mdp_state;

            LOG_COMMENT("-----------------------------------------");
            LOG_COMMENT("Running episodes with optimal kmdp-policy");
            LOG_COMMENT("-----------------------------------------");
            LOG_COMMENT("");

            LOG_COMMENT("Maze size: " << Data::maze_x_dim << "x" << Data::maze_y_dim);
            LOG_COMMENT("epsilon  = " << maze.get_epsilon() );
            LOG_COMMENT("discount = " << qiteration.get_discount() );
            LOG_COMMENT("");

            LOG_COMMENT("epsilon discount learning_length test_length mean_reward");
            LOG_COMMENT("");

            // increasing length of learning episodes
            for(unsigned long learn_length=10; learn_length<3000; learn_length*=1.25) {

                LOG_COMMENT("Learning form random episode of length " << learn_length);
                crf.clear_data(); // clear old data

                // random starting state
                maze.set_current_state(rand()%Data::state_n);
                for(unsigned long state_counter=0; state_counter<Data::k; ++state_counter) {
                    action_t random_action = rand()%Data::action_n;
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(random_action,state,reward);
                    crf.add_action_state_reward_tripel(random_action,state,reward);
                    current_k_mdp_state.new_state(random_action,state,reward);
                }
                // random transitions
                for(unsigned long learn_transition_counter=1; learn_transition_counter<=learn_length; ++learn_transition_counter) {
                    action_t random_action = rand()%Data::action_n;
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(random_action,state,reward);
                    crf.add_action_state_reward_tripel(random_action,state,reward);
                    current_k_mdp_state.new_state(random_action,state,reward);
                }

                double max_change_threshold = 0.0001, max_change;
                LOG_COMMENT("Running Q-Iteration (max change threshold = " << max_change_threshold << ")");
                unsigned long iteration_counter = 0;
                crf.initialize_kmdp_predictions(qiteration);
                do {
                    max_change = qiteration.iterate();
                    ++iteration_counter;
                    DEBUG_OUT(2,"Iteration " << iteration_counter << ": max_change = " << max_change);
                } while (max_change>max_change_threshold);

                unsigned long max_transition = 1000;
                LOG_COMMENT("Running episode of length " << max_transition << " from random starting state with optimal kmdp policy");

                double reward_sum = 0;

                // random starting state
                maze.set_current_state(rand()%Data::state_n);
                for(unsigned long state_counter=0; state_counter<Data::k; ++state_counter) {
                    action_t random_action = rand()%Data::action_n;
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(random_action,state,reward);
                    current_k_mdp_state.new_state(random_action,state,reward);
                }

                for(unsigned long transition_counter=1; transition_counter<=max_transition; ++transition_counter) {
                    action_t action = qiteration.optimal_action(current_k_mdp_state.get_k_mdp_state());
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(action,state,reward);
                    current_k_mdp_state.new_state(action,state,reward);
                    //                DEBUG_OUT(1,"Transition " << transition_counter+1 << ": " << Data::action_strings[action] << " " << state << " " << reward );
                    reward_sum += reward;
                    DEBUG_OUT(2, "Transition " << transition_counter << ": mean reward = " << reward_sum/transition_counter);
                }

                LOG( epsilon << " " << discount << " " << " " << crf.get_training_data_length() << " " << max_transition << " " << reward_sum/max_transition );

            }

            DEBUG_OUT(1,"");
            DEBUG_OUT(1,"DONE");
            DEBUG_OUT(1,"");

            log_file.close();
        } else if(arg_1=="sparse") {

            //----------------------------------//
            //    Learned sparse transitions    //
            //----------------------------------//

            QString log_file_name("log_file_sparse_");
            log_file_name.append(QString::number(epsilon));
            log_file_name.append("_");
            log_file_name.append(QString::number(discount));
            log_file_name.append("_");
            log_file_name.append(QString::number(rand()));
            log_file_name.append(".txt");
            std::ofstream log_file;
            log_file.open((const char*)log_file_name.toAscii());

            Maze maze(epsilon);
            QIteration qiteration(discount);
            KMarkovCRF crf;
            KMDPState current_k_mdp_state;

            LOG_COMMENT("-------------------------------------------");
            LOG_COMMENT("Running episodes with optimal sparse policy");
            LOG_COMMENT("-------------------------------------------");
            LOG_COMMENT("");

            LOG_COMMENT("Maze size: " << Data::maze_x_dim << "x" << Data::maze_y_dim);
            LOG_COMMENT("epsilon  = " << maze.get_epsilon() );
            LOG_COMMENT("discount = " << qiteration.get_discount() );
            LOG_COMMENT("");

            LOG_COMMENT("epsilon discount learning_length test_length l1-factor number_of_features mean_data_likelihood mean_reward");
            LOG_COMMENT("");

            // increasing length of learning episodes
            for(unsigned long learn_length=10; learn_length<3000; learn_length*=1.25) {

                LOG_COMMENT("Learning form random episode of length " << learn_length);
                crf.clear_data(); // clear old data

                // random starting state
                maze.set_current_state(rand()%Data::state_n);
                for(unsigned long state_counter=0; state_counter<Data::k; ++state_counter) {
                    action_t random_action = rand()%Data::action_n;
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(random_action,state,reward);
                    crf.add_action_state_reward_tripel(random_action,state,reward);
                    current_k_mdp_state.new_state(random_action,state,reward);
                }
                // random transitions
                for(unsigned long learn_transition_counter=1; learn_transition_counter<=learn_length; ++learn_transition_counter) {
                    action_t random_action = rand()%Data::action_n;
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(random_action,state,reward);
                    crf.add_action_state_reward_tripel(random_action,state,reward);
                    current_k_mdp_state.new_state(random_action,state,reward);
                }

                int ret;

                // learn sparse model
                crf.erase_all_features();                       // erase all features from last iteration
                crf.score_features_by_gradient(1);              // score single
                crf.add_compound_features_to_active(0);         // add single
                ret = crf.optimize_model(0,0);                  // optimize
                crf.score_features_by_gradient(1);              // score pairs
                crf.add_compound_features_to_active(0);         // add pairs

                if(ret!=0 && ret!=2) {
                    LOG_COMMENT("Error: lbfgs returned " << ret);
                }

                // increase l1 regularization to decrease number of features
                for(double l1=0.0; l1<=0.05; l1*=1.25) {

                    LOG_COMMENT("Using l1-factor of " << l1);

                    ret = crf.optimize_model(l1,0); // optimize with l1 reqularization
                    if(ret!=0 && ret!=2) {
                        LOG_COMMENT("Error: lbfgs returned " << ret);
                    }

                    crf.erase_zero_features();      // erase zero features
                    unsigned int number_of_features = crf.get_number_of_features();

                    double mean_likelihood = 0;
                    ret = crf.optimize_model(0,0,&mean_likelihood); // optimize WITHOUT l1 reqularization
                    if(ret!=0 && ret!=2) {
                        LOG_COMMENT("Error: lbfgs returned " << ret);
                    }

                    // evaluate
                    double max_change_threshold = 0.0001, max_change;
                    LOG_COMMENT("Running Q-Iteration (max change threshold = " << max_change_threshold << ")");
                    unsigned long iteration_counter = 0;
                    crf.initialize_sparse_predictions(qiteration);
                    do {
                        max_change = qiteration.iterate();
                        ++iteration_counter;
                        DEBUG_OUT(2,"Iteration " << iteration_counter << ": max_change = " << max_change);
                    } while (max_change>max_change_threshold);

                    unsigned long max_transition = 1000;
                    LOG_COMMENT("Running episode of length " << max_transition << " from random starting state with optimal sparse policy");

                    double reward_sum = 0;

                    // random starting state
                    maze.set_current_state(rand()%Data::state_n);
                    for(unsigned long state_counter=0; state_counter<Data::k; ++state_counter) {
                        action_t random_action = rand()%Data::action_n;
                        state_t state;
                        reward_t reward;
                        maze.perform_transition(random_action,state,reward);
                        current_k_mdp_state.new_state(random_action,state,reward);
                    }

                    for(unsigned long transition_counter=1; transition_counter<=max_transition; ++transition_counter) {
                        action_t action = qiteration.optimal_action(current_k_mdp_state.get_k_mdp_state());
                        state_t state;
                        reward_t reward;
                        maze.perform_transition(action,state,reward);
                        current_k_mdp_state.new_state(action,state,reward);
                        //                DEBUG_OUT(1,"Transition " << transition_counter+1 << ": " << Data::action_strings[action] << " " << state << " " << reward );
                        reward_sum += reward;
                        DEBUG_OUT(2, "Transition " << transition_counter << ": mean reward = " << reward_sum/transition_counter);
                    }

                    LOG( epsilon << " " << discount << " " << " " << crf.get_training_data_length() << " " << max_transition << " " << " " << l1 << " " << number_of_features << " " << mean_likelihood << " " << reward_sum/max_transition );

                    if(l1==0.0) l1=0.001;
                }
            }

            DEBUG_OUT(1,"");
            DEBUG_OUT(1,"DONE");
            DEBUG_OUT(1,"");

            log_file.close();
        } else {
            DEBUG_OUT(1, "Unknown command. Expecting one of");
            DEBUG_OUT(1, "    optimal, kmdp, sparse");
            DEBUG_OUT(1, "Terminating...");
            return 1;
        }
    }
    return 0;
}
