#include "BatchMaze.h"

#include "util.h"
#include "Maze.h"
#include "KMarkovCRF.h"
#include "LookAheadSearch.h"
#include "UTree.h"
#include "LinearQ.h"
#include "SmoothingKernelSigmoid.h"

#include <omp.h>

#include "qcustomplot.h"
#include <QApplication>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <float.h> // for DBL_MAX
#include <set>
#include <tuple>

#include <QString>

#define DEBUG_LEVEL 0
#include "debug.h"

#define LOG_COMMENT(x) DEBUG_OUT(1,x); log_file << "# " << x << std::endl;
#define LOG(x) DEBUG_OUT(1,x); log_file << x << std::endl;

#define RUN_ACTIVE

//#define USE_OMP

using std::set;
using std::tuple;
using std::make_tuple;
using std::get;

static QString file_name_base;
static std::ofstream log_file;
static enum OPTION { RANDOM, OPTIMAL, SPARSE, UTREE_PROB, UTREE_VALUE, LINEAR_Q, OPTION_N } option;
static const char * option_arr[OPTION_N] = {"random", "optimal", "sparse", "utree-prob", "utree-value", "linear-q"};
static QString option_str;
static int max_episodes = 100;
static int max_transitions = 1000;
static double epsilon = 0.0;
static double discount = 0.5;
static int max_training_length = 10000;
static int min_training_length = 100;
static double training_length_factor = 2;
static double training_length_incr = 0;
static double l1_factor = 0.0001;
static int max_tree_size = 10000;
static int feature_complx = 2;

BatchMaze::BatchMaze(): file_id(10000001) {}

BatchMaze::~BatchMaze() {}

int BatchMaze::run(int argn, char ** argarr) {
#ifdef RUN_ACTIVE
    return run_active(argn,argarr);
#else
    return run_predefined(argn,argarr);
#endif
}

int BatchMaze::run_predefined(int argn, char ** argarr) {

    //------------------------//
    // check for valid option //
    //------------------------//

    bool valid_option = false;
    if(argn>=1) {
        option_str = argarr[1];
        for(int op_idx=0; op_idx<OPTION_N; ++op_idx) {
            if(option_str==option_arr[op_idx]) {
                valid_option = true;
                option = (OPTION)op_idx;
                break;
            }
        }
    }

    // terminate if no valid option was passed
    if(!valid_option) {
        print_help();
        return 0;
    }

    // parse command line arguments
    parse_command_line_arguments(argn,argarr);

    // initialize log file
    initialize_log_file();

    //-----------------------------//
    // precompute training lengths //
    //-----------------------------//
    int * training_lengths;
    int training_steps;
    precompute_training_lengths(training_lengths, training_steps);

    //--------------//
    // run episodes //
    //--------------//
    double global_reward_sum = 0;
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(2)
#endif
    for(int training_idx=0; training_idx<training_steps; ++training_idx) {
        for(int episode_counter=1; episode_counter<=max_episodes; ++episode_counter) {

            // use pointers to serialize initialization
            Maze * maze;
            instance_t * current_instance;
            LookAheadSearch * look_ahead_search;
            KMarkovCRF * crf;
            UTree * utree;
            LinearQ * linQ;

#ifdef USE_OMP
#pragma omp critical
#endif
            {
                // initialize maze
                state_t start_state = state_t::random_state();
                maze = new Maze(epsilon);
                maze->set_current_state(start_state);
                current_instance = instance_t::create(action_t::STAY,start_state,reward_t(0));

                // initialize look ahead search
                look_ahead_search = new LookAheadSearch(discount);

                // initialize learners
                crf = new KMarkovCRF();
                utree = new UTree(discount);
                linQ = new LinearQ(discount);

                // initialize to minimal length history
                for(unsigned long state_counter=0; state_counter<Config::k; ++state_counter) {
                    action_t action = action_t::STAY;
                    state_t state;
                    reward_t reward;
                    maze->perform_transition(action,state,reward);
                    current_instance = current_instance->append_instance(action,state,reward);
                }

                // get training data
                for(int train_step=0; train_step<training_lengths[training_idx]; ++train_step) {
                    action_t action = action_t::random_action();
                    state_t state;
                    reward_t reward;
                    maze->perform_transition(action,state,reward);
                    current_instance = current_instance->append_instance(action,state,reward);
                    if(option==OPTIMAL || option==RANDOM) {
                        // no training data
                    } else if(option==SPARSE) {
                        crf->add_action_state_reward_tripel(action,state,reward,false);
                    } else if(option==UTREE_VALUE || option==UTREE_PROB) {
                        utree->add_action_state_reward_tripel(action,state,reward,false);
                    } else if(option==LINEAR_Q) {
                        linQ->add_action_state_reward_tripel(action,state,reward,false);
                    } else {
                        DEBUG_DEAD_LINE;
                    }
                }
            }

            // train the learners
            if(option==OPTIMAL || option==RANDOM) {
                // nothing to train
            } else if(option==SPARSE) {
                for(int complx=1; complx<=feature_complx; ++complx) {
                    crf->score_features_by_gradient(1);
                    crf->sort_scored_features(false);
                    crf->add_candidate_features_to_active(0);
                    crf->optimize_model(complx==1?0:l1_factor,0,nullptr); // no l1 in first run
                    crf->erase_zero_features();
                }
                // finalize
                crf->optimize_model(0,0,nullptr);
            } else if(option==UTREE_PROB) {
                utree->set_expansion_type(UTree::STATE_REWARD_EXPANSION);
                double score_threshold = 1e-3;
                double max_score = DBL_MAX;
                while(max_score>score_threshold) {
                    max_score = utree->expand_leaf_node(score_threshold);
                }
            } else if(option==UTREE_VALUE) {
                utree->set_expansion_type(UTree::UTILITY_EXPANSION);
                double score_threshold = 1e-3;
                double max_score = DBL_MAX;
                while(max_score>score_threshold) {
                    double max_update = DBL_MAX;
                    while(max_update>1e-10) {
                        max_update = utree->value_iteration();
                    }
                    max_score = utree->expand_leaf_node(score_threshold);
                }
            } else if(option==LINEAR_Q) {
                for(int complx=1; complx<=feature_complx; ++complx) {
                    linQ->add_candidates(1);
                    linQ->erase_zero_features();
                    linQ->optimize_l1(l1_factor);
                    linQ->erase_zero_weighted_features();
                }
                // finalize
                linQ->optimize_ridge(1e-10);
            } else {
                DEBUG_DEAD_LINE;
            }

            // perform transitions
            double reward_sum = 0;
            int transition_counter;
            for(transition_counter=1; transition_counter<=max_transitions; ++transition_counter) {

                // transition variables
                action_t action;
                state_t state;
                reward_t reward;

                // choose the action
                if(option==OPTIMAL) {
                    if(look_ahead_search->get_number_of_nodes()==0) {
                        look_ahead_search->build_tree<Maze>(
                            current_instance,
                            *maze,
                            max_tree_size
                            );
                    } else {
                        look_ahead_search->fully_expand_tree<Maze>(
                            *maze,
                            max_tree_size
                            );
                    }
                    action = look_ahead_search->get_optimal_action();
                } else if(option==RANDOM) {
                    action = action_t::random_action();
                } else if(option==SPARSE) {
                    look_ahead_search->clear_tree();
                    look_ahead_search->build_tree<KMarkovCRF>(
                        current_instance,
                        *crf,
                        max_tree_size
                        );
                    action = look_ahead_search->get_optimal_action();
                } else if(option==UTREE_PROB) {
                    look_ahead_search->clear_tree();
                    look_ahead_search->build_tree<UTree>(
                        current_instance,
                        *utree,
                        max_tree_size
                        );
                    action = look_ahead_search->get_optimal_action();
                } else if(option==UTREE_VALUE) {
                    action = utree->get_max_value_action(current_instance);
                } else if(option==LINEAR_Q) {
                    action = linQ->get_max_value_action(current_instance);
                } else {
                    DEBUG_DEAD_LINE;
                }

                // perform transition
                maze->perform_transition(action,state,reward);
                current_instance = current_instance->append_instance(action,state,reward);

                // prune search tree
                if(option==OPTIMAL) {
                    look_ahead_search->prune_tree(action,current_instance,*maze);
                } else if(option==SPARSE) {
                    look_ahead_search->prune_tree(action,current_instance,*crf);
                } else if(option==UTREE_PROB) {
                    look_ahead_search->prune_tree(action,current_instance,*utree);
                } else if(option==RANDOM || option==UTREE_VALUE || option==LINEAR_Q) {
                    // no search tree
                } else {
                    DEBUG_DEAD_LINE;
                }

                // increment reward
                reward_sum += reward;

                DEBUG_OUT(2, "Episode " << episode_counter <<
                          ",	training length " << training_lengths[training_idx] <<
                          ",	transition " << transition_counter <<
                          ":	current mean reward = " << reward_sum/transition_counter
                    );
            }
#ifdef USE_OMP
#pragma omp critical
#endif
            {
                // update global reward sum
                global_reward_sum += reward_sum;

                // write data to log file
                LOG(episode_counter << " 	" <<
                    training_lengths[training_idx] << "	" <<
                    (option==SPARSE ? crf->get_number_of_features() : 0) << "	" <<
                    ( (option==UTREE_VALUE || option==UTREE_PROB) ? utree->get_tree_size() : 0) << "	" <<
                    reward_sum/max_transitions
                    );

                // delete pointers
                delete maze;
                delete current_instance;
                delete look_ahead_search;
                delete crf;
                delete utree;
                delete linQ;
            }
        }
    }

    delete training_lengths;

    LOG_COMMENT("Global mean reward " << global_reward_sum/(max_episodes*max_transitions*training_steps));
    log_file.close();

    return 0;
}

int BatchMaze::run_active(int argn, char ** argarr) {

    //------------------------//
    // check for valid option //
    //------------------------//

    bool valid_option = false;
    if(argn>=1) {
        option_str = argarr[1];
        for(int op_idx=0; op_idx<OPTION_N; ++op_idx) {
            if(option_str==option_arr[op_idx]) {
                valid_option = true;
                option = (OPTION)op_idx;
                break;
            }
        }
    }

    // terminate if no valid option was passed
    if(!valid_option) {
        print_help();
        return 0;
    }

    // parse command line arguments
    parse_command_line_arguments(argn,argarr);

    // initialize log file
    initialize_log_file();

    //--------------------------------------------//
    // setup smoothing kernel for active sampling //
    //--------------------------------------------//
    // smoothing kernel containing the actual data points
    SmoothingKernelSigmoid sks(training_length_incr,  // use as kernel width
                               2,                     // squared exponential
                               min_training_length,   // lower bound for sampling region
                               max_training_length    // upper bound for sampling region
        );
    // store virtual data points for parallelization
    set<tuple<int,int,double> > virtual_data;

    //--------------------------//
    // run episodes in parallel //
    //--------------------------//
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(2)
#endif
    for(int episode_counter=1; episode_counter<=max_episodes; ++episode_counter) {
        double virtual_reward;
        int training_length;

        // use pointers to serialize initialization
        Maze             * maze                 = nullptr;
        instance_t       * current_instance     = nullptr;
        LookAheadSearch  * look_ahead_search    = nullptr;
        KMarkovCRF       * crf                  = nullptr;
        UTree            * utree                = nullptr;
        LinearQ          * linQ                 = nullptr;

#ifdef USE_OMP
#pragma omp critical
#endif
        {
            // initialize maze
            state_t start_state = state_t::random_state();
            maze = new Maze(epsilon);
            maze->set_current_state(start_state);
            current_instance = instance_t::create(action_t::STAY,start_state,reward_t(0));

            // initialize look ahead search
            look_ahead_search = new LookAheadSearch(discount);

            // initialize learners
            crf = new KMarkovCRF();
            utree = new UTree(discount);
            linQ = new LinearQ(discount);

            // initialize to minimal length history
            for(unsigned long state_counter=0; state_counter<Config::k; ++state_counter) {
                action_t action = action_t::STAY;
                state_t state;
                reward_t reward;
                maze->perform_transition(action,state,reward);
                current_instance = current_instance->append_instance(action,state,reward);
            }

            // get training length
            SmoothingKernelSigmoid virtual_sks = sks;
            for(auto data_point : virtual_data) {
                virtual_sks.add_new_point(get<1>(data_point),get<2>(data_point));
            }
            training_length = round(virtual_sks.get_max_uncertain(max_training_length-min_training_length));
            training_length += rand()%5-2; // +/- two steps of noise
            training_length = util::clamp<int>(min_training_length,max_training_length,training_length);
            double dummy_dev;
            virtual_sks.mean_dev(training_length,virtual_reward,dummy_dev);
            virtual_data.insert(make_tuple(episode_counter,training_length,virtual_reward));

            // generate training data
            for(int train_step=0; train_step<training_length; ++train_step) {
                action_t action = action_t::random_action();
                state_t state;
                reward_t reward;
                maze->perform_transition(action,state,reward);
                current_instance = current_instance->append_instance(action,state,reward);
                if(option==OPTIMAL || option==RANDOM) {
                    // no training data
                } else if(option==SPARSE) {
                    crf->add_action_state_reward_tripel(action,state,reward,false);
                } else if(option==UTREE_VALUE || option==UTREE_PROB) {
                    utree->add_action_state_reward_tripel(action,state,reward,false);
                } else if(option==LINEAR_Q) {
                    linQ->add_action_state_reward_tripel(action,state,reward,false);
                } else {
                    DEBUG_DEAD_LINE;
                }
            }
        }

        // train the learner
        if(option==OPTIMAL || option==RANDOM) {
            // nothing to train
        } else if(option==SPARSE) {
            for(int complx=1; complx<=feature_complx; ++complx) {
                crf->score_features_by_gradient(1);
                crf->sort_scored_features(false);
                crf->add_candidate_features_to_active(0);
                crf->optimize_model(complx==1?0:l1_factor,0,nullptr); // no l1 in first run
                crf->erase_zero_features();
            }
            // finalize
            crf->optimize_model(0,0,nullptr);
        } else if(option==UTREE_PROB) {
            utree->set_expansion_type(UTree::STATE_REWARD_EXPANSION);
            double score_threshold = 1e-3;
            double max_score = DBL_MAX;
            while(max_score>score_threshold) {
                max_score = utree->expand_leaf_node(score_threshold);
            }
        } else if(option==UTREE_VALUE) {
            utree->set_expansion_type(UTree::UTILITY_EXPANSION);
            double score_threshold = 1e-3;
            double max_score = DBL_MAX;
            while(max_score>score_threshold) {
                double max_update = DBL_MAX;
                while(max_update>1e-10) {
                    max_update = utree->value_iteration();
                }
                max_score = utree->expand_leaf_node(score_threshold);
            }
        } else if(option==LINEAR_Q) {
            for(int complx=1; complx<=feature_complx; ++complx) {
                linQ->add_candidates(1);
                linQ->erase_zero_features();
                linQ->optimize_l1(l1_factor);
                linQ->erase_zero_weighted_features();
            }
            // finalize
            linQ->optimize_ridge(1e-10);
        } else {
            DEBUG_DEAD_LINE;
        }

        // perform transitions
        double reward_sum = 0;
        int transition_counter;
        for(transition_counter=1; transition_counter<=max_transitions; ++transition_counter) {

            // transition variables
            action_t action;
            state_t state;
            reward_t reward;

            // choose the action
            if(option==OPTIMAL) {
                if(look_ahead_search->get_number_of_nodes()==0) {
                    look_ahead_search->build_tree<Maze>(
                        current_instance,
                        *maze,
                        max_tree_size
                        );
                } else {
                    look_ahead_search->fully_expand_tree<Maze>(
                        *maze,
                        max_tree_size
                        );
                }
                action = look_ahead_search->get_optimal_action();
            } else if(option==RANDOM) {
                action = action_t::random_action();
            } else if(option==SPARSE) {
                look_ahead_search->clear_tree();
                look_ahead_search->build_tree<KMarkovCRF>(
                    current_instance,
                    *crf,
                    max_tree_size
                    );
                action = look_ahead_search->get_optimal_action();
            } else if(option==UTREE_PROB) {
                look_ahead_search->clear_tree();
                look_ahead_search->build_tree<UTree>(
                    current_instance,
                    *utree,
                    max_tree_size
                    );
                action = look_ahead_search->get_optimal_action();
            } else if(option==UTREE_VALUE) {
                action = utree->get_max_value_action(current_instance);
            } else if(option==LINEAR_Q) {
                action = linQ->get_max_value_action(current_instance);
            } else {
                DEBUG_DEAD_LINE;
            }

            // perform transition
            maze->perform_transition(action,state,reward);
            current_instance = current_instance->append_instance(action,state,reward);

            // prune search tree
            if(option==OPTIMAL) {
                look_ahead_search->prune_tree(action,current_instance,*maze);
            } else if(option==SPARSE) {
                look_ahead_search->prune_tree(action,current_instance,*crf);
            } else if(option==UTREE_PROB) {
                look_ahead_search->prune_tree(action,current_instance,*utree);
            } else if(option==RANDOM || option==UTREE_VALUE || option==LINEAR_Q) {
                // no search tree
            } else {
                DEBUG_DEAD_LINE;
            }

            // increment reward
            reward_sum += reward;

            DEBUG_OUT(2, "Episode " << episode_counter <<
                      ",	training length " << training_length <<
                      ",	transition " << transition_counter <<
                      ":	current mean reward = " << reward_sum/transition_counter
                );
        }
#ifdef USE_OMP
#pragma omp critical
#endif
        {

            // write data to log file
            LOG(episode_counter << " 	" <<
                training_length << "	" <<
                (option==SPARSE ? crf->get_number_of_features() : 0) << "	" <<
                ( (option==UTREE_VALUE || option==UTREE_PROB) ? utree->get_tree_size() : 0) << "	" <<
                reward_sum/max_transitions
                );

            // update smoothing kernel and virtual data
            bool success = virtual_data.erase(make_tuple(episode_counter,training_length,virtual_reward));
            if(!success) {
                DEBUG_OUT(0,"Error: Could not remove virtual data point (" << episode_counter << "," <<
                          training_length << "," <<
                          virtual_reward << ")"
                    );
            }
            sks.add_new_point(training_length,reward_sum/max_transitions);

            // print graph to file
            QString plot_file_name = file_name_base;
            plot_file_name.prepend("print_file_");
            plot_file_name.append(QString("_%1.png").arg(QString::number(episode_counter),(int)floor(log10(max_episodes)+1),QChar('0')));
            // QApplication a(argn, argarr);
            // QWidget * w = new QWidget();
            // QCustomPlot * plotter = new QCustomPlot(w);
            // plotter->savePng(plot_file_name,1000,700,1,-1);
            // // plotter->savePdf(plot_file_name,true,1000,700);
            // delete plotter;

            // delete pointers
            delete maze;
            delete current_instance;
            delete look_ahead_search;
            delete crf;
            delete utree;
            delete linQ;
        }
    }

    // close log file
    log_file.close();

    return 0;
}

void BatchMaze::print_help() {
    DEBUG_OUT(0,"No valid option given. Please use one of:");
    for( const char * opt : option_arr ) {
        DEBUG_OUT(0,"    " << opt);
    }
    DEBUG_OUT(0,"Switches:");
    DEBUG_OUT(0,"    -e          <double>    set epsilon");
    DEBUG_OUT(0,"    -d          <double>    set discount");
    DEBUG_OUT(0,"    -maxEp      <int>       set number of episodes");
    DEBUG_OUT(0,"    -maxTran    <int>       set length of episode / number of transition");
    DEBUG_OUT(0,"    -mintl      <int>       set minimum length of training data");
    DEBUG_OUT(0,"    -maxtl      <int>       set maximum length of training data");
    DEBUG_OUT(0,"    -tlf        <double>    set factor to increase length of training data");
    DEBUG_OUT(0,"    -tli        <int>       set increment to increase length of training data");
    DEBUG_OUT(0,"    -l1         <double>    set coefficient for L1 regularization");
    DEBUG_OUT(0,"    -maxtree    <int>       set maximum size of search tree");
    DEBUG_OUT(0,"    -fcomplx    <int>       set feature complexity for CRF and Linear-Q");
}

void BatchMaze::parse_command_line_arguments(int argn, char ** argarr) {
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
        } else if(arg_switch=="-mintl" && arg_idx<argn-1) {
            int tmp_int;
            if(util::arg_int(argarr[arg_idx+1],0,tmp_int)) {
                min_training_length = tmp_int;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-mintl requires <int> as argument");
            }
        } else if(arg_switch=="-maxtl" && arg_idx<argn-1) {
            int tmp_int;
            if(util::arg_int(argarr[arg_idx+1],0,tmp_int)) {
                max_training_length = tmp_int;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-maxTran requires <int> as argument");
            }
        } else if(arg_switch=="-tlf" && arg_idx<argn-1) {
            double tmp_double;
            if(util::arg_double(argarr[arg_idx+1],0,tmp_double)) {
                training_length_factor = tmp_double;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-tlf requires <double> as argument");
            }
        } else if(arg_switch=="-tli" && arg_idx<argn-1) {
            int tmp_int;
            if(util::arg_int(argarr[arg_idx+1],0,tmp_int)) {
                training_length_incr = tmp_int;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-tli requires <int> as argument");
            }
        } else if(arg_switch=="-l1" && arg_idx<argn-1) {
            double tmp_double;
            if(util::arg_double(argarr[arg_idx+1],0,tmp_double)) {
                l1_factor = tmp_double;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-l1 requires <double> as argument");
            }
        } else if(arg_switch=="-maxtree" && arg_idx<argn-1) {
            int tmp_int;
            if(util::arg_int(argarr[arg_idx+1],0,tmp_int)) {
                max_tree_size = tmp_int;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-maxtree requires <int> as argument");
            }
        } else if(arg_switch=="-fcomplx" && arg_idx<argn-1) {
            int tmp_int;
            if(util::arg_int(argarr[arg_idx+1],0,tmp_int)) {
                feature_complx = tmp_int;
                ++arg_idx;
            } else {
                DEBUG_OUT(0, "-fcomplx requires <int> as argument");
            }
        } else {
            DEBUG_OUT(0,"Invalid switch " << (const char*)arg_switch.toLatin1());
        }
    }
}

void BatchMaze::initialize_log_file() {
    // base name
    file_name_base = QString("");
    file_name_base.append(option_str);
    file_name_base.append("_");
    file_name_base.append(QString::number(time(nullptr)));

    // log file name
    QString log_file_name = file_name_base;
    log_file_name.prepend("log_file_");
    log_file_name.append(".txt");
    log_file.open((const char*)log_file_name.toLatin1());

    std::string tmp_reward_str = Maze::get_rewards(), reward_str;
    std::string tmp_wall_str = Maze::get_walls(), wall_str;
    for( auto c : tmp_reward_str ) {
        if(c!='\n') {
            reward_str += c;
        } else {
            reward_str += "\n# ";
        }
    }
    for( auto c : tmp_wall_str ) {
        if(c!='\n') {
            wall_str += c;
        } else {
            wall_str += "\n# ";
        }
    }

    LOG_COMMENT("Maze size:               " << Config::maze_x_size << "x" << Config::maze_y_size);
    LOG_COMMENT("strategy               = " << (const char*)option_str.toLatin1() );
    LOG_COMMENT("epsilon                = " << epsilon );
    LOG_COMMENT("discount               = " << discount );
    LOG_COMMENT("episodes               = " << max_episodes );
    LOG_COMMENT("transitions            = " << max_transitions );
    LOG_COMMENT("min training length    = " << min_training_length );
    LOG_COMMENT("max training length    = " << max_training_length );
    LOG_COMMENT("training length factor = " << training_length_factor );
    LOG_COMMENT("training length incr.  = " << training_length_incr );
    LOG_COMMENT("L1 coefficient         = " << l1_factor );
    LOG_COMMENT("max tree size          = " << max_tree_size );
    LOG_COMMENT("feature complexity     = " << feature_complx );
    LOG_COMMENT("");
    LOG_COMMENT(reward_str);
    LOG_COMMENT(wall_str);
    LOG_COMMENT("");
    LOG_COMMENT("Episode	training_length	feature_n	utree_size	episode_mean_reward");
    LOG_COMMENT("");
}

void BatchMaze::precompute_training_lengths(int * & training_lengths, int & training_steps) {
    training_steps = 0;
    for(int tmp_training_length=min_training_length;
        tmp_training_length<=max_training_length;
        tmp_training_length=tmp_training_length*training_length_factor+training_length_incr) {
        ++training_steps;
    }
    training_lengths = (int*)malloc(training_steps*sizeof(int));
    int training_idx = 0;
    for(int tmp_training_length=min_training_length;
        tmp_training_length<=max_training_length;
        tmp_training_length=tmp_training_length*training_length_factor+training_length_incr) {
        training_lengths[training_idx] = tmp_training_length;
        ++training_idx;
    }
}
