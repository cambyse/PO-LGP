#include "BatchWorker.h"

#include <util/util.h>
#include <util/QtUtil.h>
#include <environment/Environment.h>
#include <environment/Predictor.h>
#include <environment/HistoryObserver.h>
#include <CheeseMaze/CheeseMaze.h>
#include <ButtonWorld/SeparateButtonWorld.h>
#include <ButtonWorld/JointButtonWorld.h>
#include <Maze/Maze.h>
#include <learner/TemporallyExtendedModel.h>
#include <learner/TemporallyExtendedLinearQ.h>
#include <learner/UTree.h>
#include <learner/ConjunctiveAdjacency.h>
#include <planning/LookAheadSearch.h>
#include <representation/DoublyLinkedInstance.h>

#include <unistd.h>

#include <QDateTime>

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 2
#endif
#define DEBUG_STRING "BatchWorker: "
#include <util/debug.h>

// #ifdef USE_OMP
//     #undef USE_OMP
// #endif

#define LOG_COMMENT(x) DEBUG_OUT(2,x); log_file << "# " << x << std::endl;
#define LOG(x) DEBUG_OUT(2,x); log_file << x << std::endl;

typedef TemporallyExtendedModel TEM;
typedef TemporallyExtendedLinearQ TEL;

using std::string;
using std::vector;
using std::shared_ptr;
using std::make_shared;
using std::dynamic_pointer_cast;

using util::INVALID;

const vector<string> BatchWorker::mode_vector = {"DRY",
                                                 "RANDOM",
                                                 "OPTIMAL",
                                                 "TEM",
                                                 "TEL",
                                                 "MODEL_BASED_UTREE",
                                                 "VALUE_BASED_UTREE",
                                                 "SEARCH_TREE",
                                                 "TRANSITIONS"};

BatchWorker::BatchWorker(int argc, char ** argv):
    mode_arg(          "m", "mode"         , "mode to use"                                  , true,        "",  "string"),
    environment_arg(    "", "env"          , "environment to use"                           ,false, "Minimal",  "string"),
    minT_arg(           "", "minT"         , "minimum number of training samples"           , true,        10,     "int"),
    maxT_arg(           "", "maxT"         , "maximum number of training samples"           ,false,        -1,     "int"),
    incT_arg(           "", "incT"         , "increment of training samples"                ,false,        -1,     "int"),
    eval_arg(          "e", "eval"         , "length of evaluation episode"                 , true,        10,     "int"),
    repeat_arg(        "r", "repeat"       , "how many times to repeat everything"          ,false,         1,     "int"),
    discount_arg(      "d", "discount"     , "discount"                                     ,false,       0.5,  "double"),
    tree_arg(          "t", "tree"         , "maximum size of search tree"                  ,false,     10000,     "int"),
    l1_arg(             "", "l1"           , "L1-regularization factor"                     ,false,     0.001,  "double"),
    pruningOff_arg(    "p", "pruningOff"   , "whether to turn off pruning the search tree"  ,           false           ),
    minH_arg(           "", "minH"         , "minimum horizon"                              ,false,        -1,     "int"),
    maxH_arg(           "", "maxH"         , "maximum horizon"                              ,false,        -1,     "int"),
    extH_arg(           "", "extH"         , "horizon extension"                            ,false,         1,     "int"),
    delta_arg(         "D", "delta"        , "minimum change of data likelihood/TD-error"   ,false,     0.001,  "double"),
    maxCycles_arg(      "", "maxCycles"    , "maximum number of grow-shrinc cycles"         ,false,         0,     "int"),
    minCycles_arg(      "", "minCycles"    , "minimum number of grow-shrinc cycles (negative"
                                             "values to force immediate full expansion"     ,false,         0,     "int"),
    epsilon_arg(        "", "eps"          , "epsilon / randomness of transitions"          ,false,         0,  "double"),
    button_n_arg(       "", "button_n"     , "number of buttons in button world"            ,false,         3,     "int"),
    button_alpha_arg(   "", "button_alpha" , "for beta dist. per button in button world"    ,false,      0.01,  "double"),
    utree_threshold_arg("u", "utree_threshold", "threshold for expansion of UTree", false, -1, "double")

{
    try {
	TCLAP::CmdLine cmd("This program is BatchWorker. It collects data.", ' ', "");

        cmd.add(utree_threshold_arg);
        cmd.add(button_alpha_arg);
        cmd.add(button_n_arg);
        cmd.add(epsilon_arg);
        cmd.add(minCycles_arg);
        cmd.add(maxCycles_arg);
        cmd.add(delta_arg);
        cmd.add(extH_arg);
        cmd.add(maxH_arg);
        cmd.add(minH_arg);
        cmd.add(pruningOff_arg);
        cmd.add(l1_arg);
        cmd.add(tree_arg);
        cmd.add(discount_arg);
        cmd.add(repeat_arg);
        cmd.add(eval_arg);
        cmd.add(incT_arg);
        cmd.add(maxT_arg);
        cmd.add(minT_arg);
        cmd.add(environment_arg);
        cmd.add(mode_arg);

	// TCLAP::ValueArg<std::string> nameArg("n","name","Name to print",false,"homer","string");
	// cmd.add( nameArg );
	// TCLAP::SwitchArg reverseSwitch("r","reverse","Print name backwards", cmd, false);

	// Parse the argv array (throws execption in case of failure)
	cmd.parse( argc, argv );

        args_ok = post_process_args();

    } catch (TCLAP::ArgException &e) {
        // catch any exceptions
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }
}

bool BatchWorker::post_process_args() {
    bool args_ok = true;

    // check mode
    {
        bool mode_ok = false;
        for(string s : mode_vector) {
            if(mode_arg.getValue()==s) {
                mode = mode_arg.getValue();
                mode_ok = true;
                break;
            }
        }
        if(!mode_ok) {
            DEBUG_WARNING("Argument value for '" << mode_arg.getName() << "' must be one of:");
            for(string s : mode_vector) {
                DEBUG_WARNING("    " << s);
            }
        } else {
            if(mode!="DRY" && mode!="RANDOM" && mode!="TEM" && mode!="TEL" && mode!="MODEL_BASED_UTREE" && mode!="VALUE_BASED_UTREE") {
                DEBUG_WARNING("mode '" << mode << "' currently not supported");
                mode_ok = false;
            }
        }
        args_ok = args_ok && mode_ok;
    }

    // check number of training samples
    {
        bool samples_ok = true;
        if(minT_arg.getValue()<=0) {
            DEBUG_WARNING("Argument '" << minT_arg.getName() << "' requires a value grater than zero");
            samples_ok = false;
        } else {
            minT = minT_arg.getValue();
        }
        if(maxT_arg.getValue()!=-1 && maxT_arg.getValue()<minT) {
            DEBUG_WARNING("Argument '" << maxT_arg.getName() << "' requires a value greater that that of '" << minT_arg.getName() << "'");
            samples_ok = false;
        } else {
            maxT = maxT_arg.getValue();
            maxT = maxT==-1?minT:maxT;
        }
        if(incT_arg.getValue()!=-1 && incT_arg.getValue()<0) {
            DEBUG_WARNING("Argument '" << incT_arg.getName() << "' requires a value greater that zero");
            samples_ok = false;
        } else {
            incT = incT_arg.getValue();
            incT = incT==-1?(maxT-minT):incT;
            incT = incT==0?1:incT;
        }
        args_ok = args_ok && samples_ok;
    }

    // check UTree threshold
    {
        utree_threshold = utree_threshold_arg.getValue();
        if(utree_threshold==-1) {
            utree_threshold = 1e-10;
        } else if(utree_threshold<0) {
            DEBUG_WARNING("Argument '" << utree_threshold_arg.getName() << "' requires a non-negative value");
            args_ok = false;
        }
    }

    // check evaluation length
    if(eval_arg.getValue()<=0) {
        DEBUG_WARNING("Argument '" << eval_arg.getName() << "' requires a value grater than zero");
        args_ok = false;
    }

    // check environment
    if(environment_arg.getValue()=="cheese" ||
       environment_arg.getValue()=="sep-button" ||
       environment_arg.getValue()=="joint-button" ||
       environment_arg.getValue()=="transfer") {
        // ok
    } else {
        auto maze = make_shared<Maze>();
        if(!maze->set_maze(QString(environment_arg.getValue().c_str()))) {
            DEBUG_WARNING("Argument value for '" << environment_arg.getName() << "' must be one of:");
            DEBUG_WARNING("--- Cheese Maze ---"       );
            DEBUG_WARNING("    cheese"                );
            DEBUG_WARNING("--- Button Worlds ---"     );
            DEBUG_WARNING("    sep-button"            );
            DEBUG_WARNING("    joint-button"          );
            DEBUG_WARNING("--- Transfer Learning ---" );
            DEBUG_WARNING("    transfer"              );
            DEBUG_WARNING("--- Other Mazes ---"       );
            for(auto s : maze->get_maze_list()) {
                DEBUG_WARNING("    " << s);
            }
            args_ok = false;
        }
    }

    // check alpha for button world
    if(button_alpha_arg.getValue()<=0) {
        DEBUG_WARNING("Argument for '" << button_alpha_arg.getName() << "' must be greater than zero");
        args_ok = false;
    }

    return args_ok;
}

void BatchWorker::collect_data() {
    if(!args_ok) {
        DEBUG_ERROR("Cannot collect data, arguments are invalid.");
        return;
    }

    std::ofstream log_file;
    initialize_log_file(log_file);

    int repeat_n = repeat_arg.getValue();
    int episode_counter = 1;

    // to coordinate learning and planning between threads
    std::vector<omp_lock_t> learn_locks;

#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(2)
#endif
    for(int repeat_index=1; repeat_index<=repeat_n; ++repeat_index) {
        for(int training_length=minT; training_length<=maxT; training_length+=incT) {
            DEBUG_OUT(2,"Training length: " << training_length);

            //-----------------//
            // local variables //
            //-----------------//

            int this_episode_counter;

            // environment
            shared_ptr<Environment> environment;
            // adjacency operator for TEM / TEL
            shared_ptr<ConjunctiveAdjacency> N_plus;
            // learner
            shared_ptr<HistoryObserver> learner;
            // spaces
            action_ptr_t action_space;
            observation_ptr_t observation_space;
            reward_ptr_t reward_space;
            // current instance
            instance_ptr_t current_instance;
            // data
            double mean_reward = 0;  // for all
            double data_likelihood;  // for TEM
            double TD_error;         // for TEL
            int nr_features;         // for TEM/TEL
            int cycles;              // for TEM/TEL
            int utree_size;          // for UTree
            double utree_score;      // for UTree
            QString button_prob_sum("NaN");// for button-worlds

            //------------//
            // initialize //
            //------------//

#ifdef USE_OMP
#pragma omp critical (BatchWorker)
#endif
            {

                this_episode_counter = episode_counter++;

                // init locks
                init_all_learn_locks(learn_locks);

                // initialize environment and get spaces
                if(environment_arg.getValue()=="cheese") {
                    environment = make_shared<CheeseMaze>();
                } else if(environment_arg.getValue()=="sep-button") {
                    auto but = make_shared<SeparateButtonWorld>(button_n_arg.getValue(), button_alpha_arg.getValue());
                    environment = but;
                    button_prob_sum = QString("%1").arg(but->get_p_sum(), 5, 'f', 3);
                } else if(environment_arg.getValue()=="joint-button") {
                    auto but = make_shared<JointButtonWorld>(button_n_arg.getValue(), button_alpha_arg.getValue());
                    environment = but;
                    button_prob_sum = QString("%1").arg(but->get_p_sum(), 5, 'f', 3);
                } else if(environment_arg.getValue()=="transfer") {
                    environment = make_shared<Maze>(epsilon_arg.getValue(),"Minimal");
                } else {
                    environment = make_shared<Maze>(epsilon_arg.getValue(),environment_arg.getValue().c_str());
                }
                environment->get_spaces(action_space,observation_space,reward_space);

                // initialize adjacency
                if(mode=="TEM" || mode=="TEL") {
                    N_plus = make_shared<ConjunctiveAdjacency>();
                    N_plus->adopt_spaces(*environment);
                    N_plus->set_horizon_extension(extH_arg.getValue());
                    N_plus->set_max_horizon(maxH_arg.getValue());
                    N_plus->set_min_horizon(minH_arg.getValue());
                    N_plus->set_combine_features(false);
                    N_plus->set_common_delay(true);
                }
                if(mode=="TEM") {
                    N_plus->set_t_zero_features(ConjunctiveAdjacency::T_ZERO_FEATURES::OBSERVATION_REWARD);
                }
                if(mode=="TEL") {
                    N_plus->set_t_zero_features(ConjunctiveAdjacency::T_ZERO_FEATURES::ACTION);
                }

                // initialize learner
                if(mode=="DRY" || mode=="RANDOM") {
                    // no learner
                } else if(mode=="TEM") {
                    auto tem = make_shared<TEM>(N_plus);
                    tem->adopt_spaces(*environment);
                    learner = tem;
                } else if(mode=="TEL") {
                    auto tel = make_shared<TEL>(N_plus,discount_arg.getValue());
                    tel->adopt_spaces(*environment);
                    learner = tel;
                } else if(mode=="MODEL_BASED_UTREE" || mode=="VALUE_BASED_UTREE") {
                    auto utree = make_shared<UTree>(discount_arg.getValue());
                    utree->adopt_spaces(*environment);
                    utree->set_features(*environment);
                    learner = utree;
                } else {
                    DEBUG_DEAD_LINE;
                }

            } // end omp critical

            //--------------------------------//
            // collect data and train learner //
            //--------------------------------//

            // do twice in case of tranfer learning tasks
            for(int n_runs=1;;++n_runs) {
                if(mode=="DRY" || mode=="RANDOM") {
                    // no data to collect, nothing to learn
                } else if(mode=="TEM" || mode=="TEL" || mode=="MODEL_BASED_UTREE" || mode=="VALUE_BASED_UTREE"){
                    // cast learner to observer (should always be possible)
                    auto observer = dynamic_pointer_cast<HistoryObserver>(learner);
                    if(observer==nullptr) {
                        DEBUG_DEAD_LINE;
                    } else {
                        // clear data (for the case of transfer learning)
                        observer->clear_data();
                        // collect data
                        collect_random_data(environment, observer, training_length, current_instance);
                    }
                    // train learner
                    if(mode=="TEM") {
                        // no parallel learning because it's memory intensive and
                        // internally parallel
                        set_all_learn_locks(learn_locks);
                        train_TEM(learner, data_likelihood, nr_features, cycles);
                        unset_all_learn_locks(learn_locks);
                    } else if(mode=="TEL") {
                        // no parallel learning because it's memory intensive and
                        // internally parallel
                        set_all_learn_locks(learn_locks);
                        train_TEL(learner, TD_error, nr_features, cycles);
                        unset_all_learn_locks(learn_locks);
                    } else if(mode=="MODEL_BASED_UTREE") {
                        train_model_based_UTree(learner, utree_size, utree_score);
                    } else if(mode=="VALUE_BASED_UTREE") {
                        train_value_based_UTree(learner, utree_size, utree_score);
                    } else {
                        DEBUG_DEAD_LINE;
                    }
                } else {
                    DEBUG_DEAD_LINE;
                }
                // change environment in case of transfer learning
                if(n_runs==1 && environment_arg.getValue()=="transfer") {
                    environment = make_shared<Maze>(epsilon_arg.getValue(),"Minimal_2");
                } else {
                    break;
                }
            }

            //----------//
            // evaluate //
            //----------//

            if(mode=="DRY") {
                // do nothing -- dry run
            } else if(mode=="RANDOM") {
                // perform random transitions
                DEBUG_OUT(2,"Performing random transitions:");
                repeat(eval_arg.getValue()) {
                    action_ptr_t action = action_space->random_element();
                    observation_ptr_t observation;
                    reward_ptr_t reward;
                    environment->perform_transition(action, observation, reward);
                    mean_reward += reward->get_value();
                    DEBUG_OUT(2,"    " << action << "	" << observation << "	" << reward);
                }
            } else if(mode=="TEM" || mode=="MODEL_BASED_UTREE"){
                // cast to predictor
                auto pred = dynamic_pointer_cast<Predictor>(learner);
                if(pred==nullptr) {
                    DEBUG_DEAD_LINE;
                }
                // initialize planner
                // TODO: This should be done using LookAheadPolicy as in PlannerTest.cpp
                LookAheadSearch planner(discount_arg.getValue());
                planner.set_spaces(action_space, observation_space, reward_space);
                // do planned steps
                for(int step_idx=0; step_idx<eval_arg.getValue(); ++step_idx) {
                    // don't do planning and learning in parallel so all threads
                    // can be used to pass the "learning bottelneck" quickly
                    set_this_learn_lock(learn_locks);
                    // do planning
                    action_ptr_t action;
                    if(pruningOff_arg.getValue() || step_idx==0) {
                        planner.clear_tree();
                        planner.build_tree(current_instance, *pred, tree_arg.getValue());
                    } else {
                        planner.fully_expand_tree(*pred, tree_arg.getValue());
                    }
                    // get action
                    action = planner.get_optimal_action();
                    // actually perform the transition
                    observation_ptr_t observation;
                    reward_ptr_t reward;
                    environment->perform_transition(action, observation, reward);
                    mean_reward += reward->get_value();
                    current_instance = current_instance->append(action, observation, reward);
                    DEBUG_OUT(2,"    " << action << "	" << observation << "	" << reward);
                    // prune tree
                    if(!pruningOff_arg.getValue()) {
                        planner.prune_tree(action,current_instance,*pred);
                    }
                    // unset lock
                    unset_this_learn_lock(learn_locks);
                }
            } else if(mode=="TEL"){
                // cast to policy
                auto policy = dynamic_pointer_cast<Policy>(learner);
                if(policy==nullptr) {
                    DEBUG_DEAD_LINE;
                }
                // do optimal transition
                repeat(eval_arg.getValue()) {
                    action_ptr_t action = policy->get_action(current_instance);
                    observation_ptr_t observation;
                    reward_ptr_t reward;
                    environment->perform_transition(action, observation, reward);
                    mean_reward += reward->get_value();
                    current_instance = current_instance->append(action, observation, reward);
                    DEBUG_OUT(2,"    " << action << "	" << observation << "	" << reward);
                }
            } else if(mode=="VALUE_BASED_UTREE"){
                // cast to utree
                auto utree = dynamic_pointer_cast<UTree>(learner);
                if(utree==nullptr) {
                    DEBUG_DEAD_LINE;
                }
                // do optimal transition
                repeat(eval_arg.getValue()) {
                    action_ptr_t action = utree->get_max_value_action(current_instance);
                    observation_ptr_t observation;
                    reward_ptr_t reward;
                    environment->perform_transition(action, observation, reward);
                    mean_reward += reward->get_value();
                    current_instance = current_instance->append(action, observation, reward);
                    DEBUG_OUT(2,"    " << action << "	" << observation << "	" << reward);
                }
            } else {
                DEBUG_DEAD_LINE;
            }

            // calculate mean reward
            mean_reward/=eval_arg.getValue();

            //--------------//
            // write output //
            //--------------//

#ifdef USE_OMP
#pragma omp critical (BatchWorker)
#endif
            {
                if(mode=="TEM") {
                    LOG(this_episode_counter	<< "	" <<
                        training_length	<< "	" <<
                        eval_arg.getValue()	<< "	" <<
                        mean_reward	<< "	" <<
                        data_likelihood	<< "	" <<
                        "NaN"	<< "	" <<
                        nr_features	<< "	" <<
                        l1_arg.getValue()	<< "	" <<
                        cycles	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        button_prob_sum
                        );
                } else if(mode=="TEL") {
                    LOG(this_episode_counter	<< "	" <<
                        training_length	<< "	" <<
                        eval_arg.getValue()	<< "	" <<
                        mean_reward	<< "	" <<
                        "NaN"	<< "	" <<
                        TD_error	<< "	" <<
                        nr_features	<< "	" <<
                        l1_arg.getValue()	<< "	" <<
                        cycles	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        button_prob_sum
                        );
                } else if(mode=="MODEL_BASED_UTREE" || mode=="VALUE_BASED_UTREE") {
                    LOG(this_episode_counter	<< "	" <<
                        training_length	<< "	" <<
                        eval_arg.getValue()	<< "	" <<
                        mean_reward	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        utree_score	<< "	" <<
                        utree_size	<< "	" <<
                        button_prob_sum
                        );
                } else if(mode=="DRY" || mode=="RANDOM") {
                    LOG(this_episode_counter	<< "	" <<
                        training_length	<< "	" <<
                        eval_arg.getValue()	<< "	" <<
                        mean_reward	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        "NaN"	<< "	" <<
                        button_prob_sum
                        );
                } else {
                    DEBUG_DEAD_LINE;
                }
            }

            current_instance->detach_reachable();
        }
    } // end omp parallel for

    // destroy locks
    destroy_all_learn_locks(learn_locks);
}

void BatchWorker::collect_random_data(std::shared_ptr<Environment> env,
                                      std::shared_ptr<HistoryObserver> obs,
                                      const int& length,
                                      instance_ptr_t& ins) {
    // get spaces
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    env->get_spaces(action_space,observation_space,reward_space);

    // perform random transitions
    DEBUG_OUT(2,"Performing random transitions:");
    repeat(length) {
        action_ptr_t action = action_space->random_element();
        observation_ptr_t observation;
        reward_ptr_t reward;
        env->perform_transition(action, observation, reward);
        obs->add_action_observation_reward_tripel(action, observation, reward, false);
        DEBUG_OUT(2,"    " << action << "	" << observation << "	" << reward);
        if(ins==INVALID) {
            ins = DoublyLinkedInstance::create(action, observation, reward);
        } else {
            ins = ins->append(action, observation, reward);
        }
    }
}

void BatchWorker::train_TEM(std::shared_ptr<HistoryObserver> learner, double& likelihood, int& features, int& cycles) {
    // cast to correct type
    auto tem = dynamic_pointer_cast<TEM>(learner);
    // check for cast failure
    if(tem==nullptr) {
        DEBUG_DEAD_LINE;
        return;
    }
    // optimize tem
    double old_likelihood = 0, new_likelihood = 1;
    int cycle_counter = 0, max_cycles = maxCycles_arg.getValue();
    tem->set_l1_factor(l1_arg.getValue());
    bool first_loop_iter = true;
    while(new_likelihood-old_likelihood>delta_arg.getValue() || abs(minCycles_arg.getValue())>cycle_counter) {
        if(first_loop_iter) {
            first_loop_iter = false;
        } else {
            old_likelihood = new_likelihood;
        }
        do {
            // multiple times for for negative min-cycle
            tem->grow_feature_set();
            ++cycle_counter;
        } while(-minCycles_arg.getValue()>cycle_counter);
        double neg_log_like = tem->optimize_weights_LBFGS();
        new_likelihood = exp(-neg_log_like);
        tem->shrink_feature_set();
        if(max_cycles>0 && cycle_counter>=max_cycles) {
            break;
        }
    }
    tem->set_l1_factor(0);
    double neg_log_like = tem->optimize_weights_LBFGS();
    // set outputs
    likelihood = exp(-neg_log_like);
    cycles = cycle_counter;
    features = tem->get_feature_set().size();
    // free memory
    tem->free_memory_after_learning();
}

void BatchWorker::train_TEL(std::shared_ptr<HistoryObserver> learner, double& TD_error, int& features, int& cycles) {
    // cast to correct type
    auto tel = dynamic_pointer_cast<TEL>(learner);
    // check for cast failure
    if(tel==nullptr) {
        DEBUG_DEAD_LINE;
        return;
    }
    // optimize tel
    double old_TD_error = DBL_MAX, new_TD_error = 0;
    int cycle_counter = 0, max_cycles = maxCycles_arg.getValue();
    tel->set_l1_factor(l1_arg.getValue());
    bool first_loop_iter = true;
    while(old_TD_error-new_TD_error>delta_arg.getValue() || abs(minCycles_arg.getValue())>cycle_counter) {
        if(first_loop_iter) {
            first_loop_iter = false;
        } else {
            old_TD_error = new_TD_error;
        }
        do {
            // multiple times for for negative min-cycle
            tel->grow_feature_set();
            ++cycle_counter;
        } while(-minCycles_arg.getValue()>cycle_counter);
        new_TD_error = tel->run_policy_iteration();
        tel->shrink_feature_set();
        if(max_cycles>0 && cycle_counter>=max_cycles) {
            break;
        }
    }
    // set outputs
    TD_error = tel->run_policy_iteration(false);
    cycles = cycle_counter;
    features = tel->get_feature_set().size();
    // free memory
    tel->free_memory_after_learning();
}

void BatchWorker::train_value_based_UTree(std::shared_ptr<HistoryObserver> learner, int& size, double& score) {
    // cast to correct type
    auto utree = dynamic_pointer_cast<UTree>(learner);
    // check for cast failure
    if(utree==nullptr) {
        DEBUG_DEAD_LINE;
        return;
    }
    // expand utree
    utree->set_expansion_type(UTree::UTILITY_EXPANSION);
    score = DBL_MAX;
    while(score >= utree_threshold) {
        score = utree->expand_leaf_node(utree_threshold);
    }
    // get tree size
    size = utree->get_tree_size();
}

void BatchWorker::train_model_based_UTree(std::shared_ptr<HistoryObserver> learner, int& size, double& score) {
    // cast to correct type
    auto utree = dynamic_pointer_cast<UTree>(learner);
    // check for cast failure
    if(utree==nullptr) {
        DEBUG_DEAD_LINE;
        return;
    }
    // expand utree
    utree->set_expansion_type(UTree::OBSERVATION_REWARD_EXPANSION);
    score = DBL_MAX;
    while(score >= utree_threshold ) {
        score = utree->expand_leaf_node(utree_threshold);
    }
    // get tree size
    size = utree->get_tree_size();
}

void BatchWorker::initialize_log_file(std::ofstream& log_file) {
    QString log_file_name = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh:mm:ss");
    log_file_name.append("_");
    log_file_name.append(mode.c_str());
    log_file_name.append("_log_file.txt");
    log_file.open((const char*)log_file_name.toLatin1());

    LOG_COMMENT("Mode: " << mode);
    LOG_COMMENT("Environment: " << environment_arg.getValue());
    LOG_COMMENT("Training Length: " << minT << "--" << maxT << " (step size: " << incT << ")");
    LOG_COMMENT("Evaluation Length: " << eval_arg.getValue());
    LOG_COMMENT("Repetitions: " << repeat_arg.getValue());
    LOG_COMMENT("Discount: " << discount_arg.getValue());
    LOG_COMMENT("Complexity Penalty: " << (Feature::use_complexity_penalty?"yes":"no"));
    if(mode=="TEM" || mode=="MODEL_BASED_UTREE") {
        LOG_COMMENT("Max Look-Ahead Tree Size: " << tree_arg.getValue());
        LOG_COMMENT("Pruning: " << (pruningOff_arg.getValue()?"off":"on"));
    }
    if(mode=="VALUE_BASED_UTREE" || mode=="MODEL_BASED_UTREE") {
        LOG_COMMENT("UTree Threshold: " << utree_threshold);
    }
    if(mode=="TEM" || mode=="TEL") {
        LOG_COMMENT("L1-factor: " << l1_arg.getValue());
        LOG_COMMENT("Minimum Horizon: " << minH_arg.getValue());
        LOG_COMMENT("Maximum Horizon: " << maxH_arg.getValue());
        LOG_COMMENT("Horizon Extension: " << extH_arg.getValue());
        LOG_COMMENT("Max Cycles: " << maxCycles_arg.getValue());
        LOG_COMMENT("Min Cycles: " << minCycles_arg.getValue());
    }
    if(mode=="TEM") {
        LOG_COMMENT("Likelihood Delta: " << delta_arg.getValue());
    }
    if(mode=="TEL") {
        LOG_COMMENT("TD-error Delta: " << delta_arg.getValue());
    }
    if(environment_arg.getValue() == "button") {
        LOG_COMMENT("Nr. Buttons: " << button_n_arg.getValue());
        LOG_COMMENT("Alpha for buttons: " << button_alpha_arg.getValue());
    }

    LOG_COMMENT("");
    LOG_COMMENT("Episode	training_length	evaluation_length	mean_reward	data_likelihood	TD-error	nr_features	l1_factor	cycles	utree_score	utree_size	button_p_sum");
    LOG_COMMENT("");
}

#ifdef USE_OMP
void BatchWorker::init_all_learn_locks(std::vector<omp_lock_t> & locks) {
    if(locks.size()==0) {
        DEBUG_OUT(3, "Init locks in thread " << omp_get_thread_num());
        repeat(omp_get_num_threads()) {
            locks.push_back(omp_lock_t());
            omp_init_lock(&(locks.back()));
        }
        DEBUG_OUT(3, "    " << locks.size() << " locks");
    }
}
#else
void BatchWorker::init_all_learn_locks(std::vector<omp_lock_t> &, omp_lock_t &) {}
#endif

#ifdef USE_OMP
void BatchWorker::set_all_learn_locks(std::vector<omp_lock_t> & locks) {
    DEBUG_OUT(3, "TRY set all locks in thread " << omp_get_thread_num());
    for(auto & l : locks) {
        omp_set_lock(&l);
    }
    DEBUG_OUT(3, "set all locks in thread " << omp_get_thread_num());
}
#else
void BatchWorker::set_all_learn_locks(std::vector<omp_lock_t> &, omp_lock_t &) {}
#endif

#ifdef USE_OMP
void BatchWorker::unset_all_learn_locks(std::vector<omp_lock_t> & locks) {
    for(auto & l : locks) {
        omp_unset_lock(&l);
    }
    DEBUG_OUT(3, "unset all locks in thread " << omp_get_thread_num());
}
#else
void BatchWorker::unset_all_learn_locks(std::vector<omp_lock_t> &, omp_lock_t &) {}
#endif

#ifdef USE_OMP
void BatchWorker::set_this_learn_lock(std::vector<omp_lock_t> & locks) {
    DEBUG_OUT(3, "TRY set lock for thread " << omp_get_thread_num());
    usleep(100000); // wait 100ms so a learner can acquire the lock
    omp_set_lock(&(locks[omp_get_thread_num()]));
    DEBUG_OUT(3, "set lock for thread " << omp_get_thread_num());
}
#else
void BatchWorker::set_this_learn_lock(std::vector<omp_lock_t> &) {}
#endif

#ifdef USE_OMP
void BatchWorker::unset_this_learn_lock(std::vector<omp_lock_t> & locks) {
    omp_unset_lock(&(locks[omp_get_thread_num()]));
    DEBUG_OUT(3, "unset lock for thread " << omp_get_thread_num());
}
#else
void BatchWorker::unset_this_learn_lock(std::vector<omp_lock_t> &) {}
#endif

#ifdef USE_OMP
void BatchWorker::destroy_all_learn_locks(std::vector<omp_lock_t> & locks) {
    DEBUG_OUT(3, "Destroy locks in thread " << omp_get_thread_num());
    for(auto & l : locks) {
        omp_destroy_lock(&l);
    }
    locks.clear();
}
#else
void BatchWorker::destroy_all_learn_locks(std::vector<omp_lock_t> &, omp_lock_t &) {}
#endif
