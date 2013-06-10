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
static unsigned long max_episodes = 100;
static unsigned long max_transitions = 1000;
static double epsilon = 0.0;
static double discount = 0.5;

BatchMaze::BatchMaze(): file_id(10000001) {}

BatchMaze::~BatchMaze() {}

int BatchMaze::run(int argn, char ** argarr) {

    //------------------------//
    // check for valid option //
    //------------------------//

    bool valid_option = false;
    QString option;

    if(argn>=1) {
        option = argarr[1];
        for(int op_idx=0; op_idx<option_n; ++op_idx) {
            if(option==option_str[op_idx]) {
                valid_option = true;
                break;
            }
        }
    }

    // terminate if no valid option was passed
    if(!valid_option) {
        DEBUG_OUT(0,"No valid option given. Please use one of:");
        for( const char * opt : option_str ) {
            DEBUG_OUT(0,"    " << opt);
        }
        DEBUG_OUT(0,"Switches:");
        DEBUG_OUT(0,"    -e          <double>    set epsilon");
        DEBUG_OUT(0,"    -d          <double>    set discount");
        DEBUG_OUT(0,"    -maxEp      <int>       set number of episodes");
        DEBUG_OUT(0,"    -maxTran    <int>       set length of episode / number of transition");
        return 0;
    }

    //------------------------------//
    // parse command line arguments //
    //------------------------------//
    for(int arg_idx=2; arg_idx<argn; ++arg_idx) {
        QString arg_switch = argarr[arg_idx];
        if(arg_switch=="-e" && arg_idx<argn-1) {
            double tmp_epsilon;
            if(util::arg_double(argarr[arg_idx+1],0,tmp_epsilon)) {
                epsilon = tmp_epsilon;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-e requires <double> as argument");
            }
        } else if(arg_switch=="-d" && arg_idx<argn-1) {
            double tmp_discount;
            if(util::arg_double(argarr[arg_idx+1],0,tmp_discount)) {
                discount = tmp_discount;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-d requires <double> as argument");
            }
        } else if(arg_switch=="-maxEp" && arg_idx<argn-1) {
            int tmp_int;
            if(util::arg_int(argarr[arg_idx+1],0,tmp_int)) {
                max_episodes = tmp_int;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-maxEp requires <int> as argument");
            }
        } else if(arg_switch=="-maxTran" && arg_idx<argn-1) {
            int tmp_int;
            if(util::arg_int(argarr[arg_idx+1],0,tmp_int)) {
                max_transitions = tmp_int;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-maxTran requires <int> as argument");
            }
        } else {
            DEBUG_OUT(0,"Invalid switch " << (const char*)arg_switch.toLatin1());
        }
    }

    //---------------------//
    // initialize log file //
    //---------------------//
    QString log_file_name("log_file_");
    log_file_name.append(option);
    log_file_name.append("_");
    log_file_name.append(QString::number(epsilon));
    log_file_name.append("_");
    log_file_name.append(QString::number(discount));
    log_file_name.append("_");
    log_file_name.append(QString::number(file_id++));
    log_file_name.append(".txt");
    std::ofstream log_file;
    log_file.open((const char*)log_file_name.toLatin1());

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

        // initialize learners
        KMarkovCRF crf;
        UTree utree(discount);
        LinearQ linQ(discount);

        // initializ to minimal length history
        for(unsigned long state_counter=0; state_counter<k; ++state_counter) {
            action_t action = action_t::STAY;
            state_t state;
            reward_t reward;
            maze.perform_transition(action,state,reward);
            current_instance = current_instance->append_instance(action,state,reward);
        }

        // train the learners
        if(option==option_str[0]) {
            // nothing to train
        } else {
            DEBUG_DEAD_LINE;
        }

        // perform transitions
        double reward_sum = 0;
        unsigned long transition_counter;
        for(transition_counter=1; transition_counter<=max_transitions; ++transition_counter) {

            // transition variables
            action_t action;
            state_t state;
            reward_t reward;

            //-------------------//
            // choose the action //
            //-------------------//
            if(option==option_str[0]) {
                // optimal
                look_ahead_search.clear_tree();
                look_ahead_search.build_tree<Maze>(
                    current_instance,
                    maze,
                    maze.get_prediction_ptr(),
                    10000
                    );
                action = look_ahead_search.get_optimal_action();
            } else {
                DEBUG_DEAD_LINE;
            }

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

    log_file.close();

    return 0;
}
