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
#include "util.h"

#include <iostream>
#include <fstream>

#include <QString>

#define DEBUG_LEVEL 0
#include "debug.h"

typedef Data::action_t action_t;
typedef Data::state_t state_t;
typedef Data::reward_t reward_t;

BatchMaze::BatchMaze() {}

BatchMaze::~BatchMaze() {}

int BatchMaze::run(int argc, char *argv[]) {
    if(argc <= 1) {
        DEBUG_OUT(1, "No arguments, terminating...");
        return 1;
    } else {
        QString arg_1(argv[1]);
        if(arg_1=="optimal") {

            double epsilon = 0.1;
            double discount = 0.9;
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

            QString log_file_name("log_file_");
            log_file_name.append(QString::number(epsilon));
            log_file_name.append("_");
            log_file_name.append(QString::number(discount));
            log_file_name.append(".txt");
            std::ofstream log_file;
            log_file.open((const char*)log_file_name.toAscii());

            Maze maze(epsilon);
            QIteration qiteration(discount);
            KMDPState current_k_mdp_state;

            DEBUG_OUT(1,"------------------------------------");
            DEBUG_OUT(1,"Running episodes with optimal policy");
            DEBUG_OUT(1,"------------------------------------");

            DEBUG_OUT(1,"");

            DEBUG_OUT(1,"Maze size: " << Data::maze_x_dim << "x" << Data::maze_y_dim);
            DEBUG_OUT(1,"epsilon  = " << maze.get_epsilon() );
            DEBUG_OUT(1,"discount = " << qiteration.get_discount() );

            DEBUG_OUT(1,"");

            double max_change_threshold = 0.0001, max_change;
            DEBUG_OUT(1,"Running Q-Iteration (max change threshold = " << max_change_threshold << ")");
            DEBUG_OUT(1,"");
            unsigned long iteration_counter = 0;
            maze.initialize_predictions(qiteration);
            do {
                max_change = qiteration.iterate();
                ++iteration_counter;
                DEBUG_OUT(2,"Iteration " << iteration_counter << ": max_change = " << max_change);
            } while (max_change>max_change_threshold);

            DEBUG_OUT(1,"");

            unsigned long max_episodes = 100;
            unsigned long max_transition = 1000;
            unsigned long total_transition_counter = 0;
            double reward_sum = 0;
            DEBUG_OUT(1,"Running " << max_episodes << " episodes of length " << max_transition << " from random starting state with optimal policy");
            DEBUG_OUT(1,"");
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
            log_file << epsilon << " " << discount << " " << reward_sum/total_transition_counter << std::endl;

            DEBUG_OUT(1,"");
            DEBUG_OUT(1,"DONE");
            DEBUG_OUT(1,"");

            log_file.close();
        } else {
            DEBUG_OUT(1, "Unknown command, terminating...");
            return 1;
        }
    }
    return 0;
}
