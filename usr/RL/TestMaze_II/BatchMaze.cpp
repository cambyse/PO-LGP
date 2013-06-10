#include "BatchMaze.h"

#include "util.h"
#include "Maze.h"
#include "KMarkovCRF.h"
#include "LookAheadSearch.h"
#include "UTree.h"
#include "LinearQ.h"

#include <omp.h>

#include <iostream>
#include <fstream>

#include <QString>

#define DEBUG_LEVEL 0
#include "debug.h"

#define LOG_COMMENT(x) DEBUG_OUT(1,x); log_file << "# " << x << std::endl;
#define LOG(x) DEBUG_OUT(1,x); log_file << x << std::endl;

USE_DATA_CONSTS;

static const int option_n = 5;
static const char * option_str[5] = {"optimal", "sparse", "utree-prob", "utree-value", "linear-q"};
static const unsigned long max_episodes = 100;
static const unsigned long max_transitions = 1000;
static double epsilon = 0.0;
static double discount = 0.5;

BatchMaze::BatchMaze(): file_id(10000001) {}

BatchMaze::~BatchMaze() {}

int BatchMaze::run(int argc, char *argv[]) {
    if(argc <= 1) {
        DEBUG_OUT(0,"No arguments given. Don't know what to do");
        return 1;
    } else {

        QString arg_1(argv[1]);

        // check for valid option and parse command line args
        bool valid_option = false;
        for(int op_idx=0; op_idx<option_n; ++op_idx) {
            if(arg_1==option_str[op_idx]) {
                valid_option = true;
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
                break;
            }
        }

        if(!valid_option) {
            DEBUG_OUT(0,"No valid option given. Please use one of:");
            for( const char * opt : option_str ) {
                DEBUG_OUT(0,"    " << opt);
            }
            return 0;
        }

        if(arg_1==option_str[0]) {

            //------------------------//
            //    Optimal Behavior    //
            //------------------------//

            // initialize log file
            QString log_file_name("log_file_optimal_");
            log_file_name.append(QString::number(epsilon));
            log_file_name.append("_");
            log_file_name.append(QString::number(discount));
            log_file_name.append("_");
            log_file_name.append(QString::number(file_id++));
            log_file_name.append(".txt");
            std::ofstream log_file;
            log_file.open((const char*)log_file_name.toLatin1());

            LOG_COMMENT("------------------------------------");
            LOG_COMMENT("Running episodes with optimal policy");
            LOG_COMMENT("------------------------------------");
            LOG_COMMENT("");
            LOG_COMMENT("Maze size: " << maze_x_size << "x" << maze_y_size);
            LOG_COMMENT("epsilon  = " << epsilon );
            LOG_COMMENT("discount = " << discount );
            LOG_COMMENT("");
            LOG_COMMENT("Running " << max_episodes << " episodes of length " << max_transitions );
            LOG_COMMENT("");
            LOG_COMMENT("Episode	episode_mean	global_mean");
            LOG_COMMENT("");

            // run episodes
            double global_reward_sum = 0;
#pragma omp parallel for
            for(unsigned long episode_counter=1; episode_counter<=max_episodes; ++episode_counter) {

                // initialize maze
                state_t start_state = state_t::random_state();
                Maze maze(epsilon);
                maze.set_current_state(start_state);
                instance_t * current_instance = instance_t::create(action_t::STAY,start_state,reward_t::min_reward);

                // initialize look ahead search
                LookAheadSearch look_ahead_search(discount);

                // initializ to minimal length history
                for(unsigned long state_counter=0; state_counter<k; ++state_counter) {
                    action_t action = action_t::STAY;
                    state_t state;
                    reward_t reward;
                    maze.perform_transition(action,state,reward);
                    current_instance = current_instance->append_instance(action,state,reward);
                }

                // perform transitions
                double reward_sum = 0;
                unsigned long transition_counter;
                for(transition_counter=1; transition_counter<=max_transitions; ++transition_counter) {

                    // transition variables
                    action_t action;
                    state_t state;
                    reward_t reward;

                    // choose action
                    look_ahead_search.clear_tree();
                    look_ahead_search.build_tree<Maze>(
                        current_instance,
                        maze,
                        maze.get_prediction_ptr(),
                        10000
                        );
                    action = look_ahead_search.get_optimal_action();

                    // perform transition
                    maze.perform_transition(action,state,reward);
                    current_instance = current_instance->append_instance(action,state,reward);

                    // increment reward
                    reward_sum += reward;

                    DEBUG_OUT(2, "Episode " << episode_counter <<
                              ",	transition " << transition_counter <<
                              ":	current mean reward = " << reward_sum/transition_counter
                        );
                }

                global_reward_sum += reward_sum;

                DEBUG_OUT(2, "Episode " << episode_counter <<
                          ":	mean reward = " << reward_sum/transition_counter
                    );
                LOG(episode_counter << " 	" << reward_sum/transition_counter);
            }

            DEBUG_OUT(1,"The mean reward was " << global_reward_sum/(max_episodes*max_transitions));
            LOG("	" << " 	" << global_reward_sum/(max_episodes*max_transitions));

            DEBUG_OUT(1,"");
            DEBUG_OUT(1,"DONE");
            DEBUG_OUT(1,"");

            log_file.close();
        }
        else {
            DEBUG_DEAD_LINE;
        }
    }
    return 0;
}
