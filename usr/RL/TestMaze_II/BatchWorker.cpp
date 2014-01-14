#include "BatchWorker.h"

#include "../util.h"
#include "../Environment.h"
#include "../Predictor.h"
#include "../HistoryObserver.h"
#include "../FeatureLearner.h"
#include "../CheeseMaze/CheeseMaze.h"
#include "../KMarkovCRF.h"
#include "../UTree.h"
#include "../LookAheadSearch.h"

#include <QDateTime>

#include <omp.h>

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 2
#endif
#define DEBUG_STRING "BatchWorker: "
#include "debug.h"

#define USE_OMP

#define LOG_COMMENT(x) DEBUG_OUT(2,x); log_file << "# " << x << std::endl;
#define LOG(x) DEBUG_OUT(2,x); log_file << x << std::endl;

using std::string;
using std::vector;
using std::shared_ptr;
using std::make_shared;
using std::dynamic_pointer_cast;

const vector<string> BatchWorker::mode_vector = {"DRY",
                                                 "RANDOM",
                                                 "OPTIMAL",
                                                 "CRF",
                                                 "MODEL_BASED_UTREE",
                                                 "VALUE_BASED_UTREE",
                                                 "LINEAR_Q_TD",
                                                 "LINEAR_Q_BELLMAN",
                                                 "SEARCH_TREE",
                                                 "TRANSITIONS"};

BatchWorker::BatchWorker(int argc, char ** argv):
    mode_arg(          "m", "mode"         , "mode to use"                                  , true,        "",  "string"),
//    environment_arg(   "e", "environment"  , "environment to use"                           , true,        "",  "string"),
    minT_arg(           "", "minT"         , "minimum number of training samples"           , true,        10,     "int"),
    maxT_arg(           "", "maxT"         , "maximum number of training samples"           ,false,        -1,     "int"),
    incT_arg(           "", "incT"         , "increment of training samples"                ,false,        -1,     "int"),
    eval_arg(          "e", "eval"         , "length of evaluation episode"                 , true,        10,     "int"),
    repeat_arg(        "r", "repeat"       , "how many times to repeat everything"          ,false,         1,     "int"),
    discount_arg(      "d", "discount"     , "discount"                                     ,false,       0.5,  "double"),
    tree_arg(          "t", "tree"         , "maximum size of search tree"                  ,false,     10000,     "int"),
    l1_arg(             "", "l1"           , "L1-regularization factor"                     ,false,     0.001,  "double"),
    pruningOff_arg(    "p", "pruningOff"   , "whether to turn off pruning the search tree"  ,           false           ),
    incF_arg(          "f", "incF"         , "how many candidate features to include"       ,false,        50,     "int"),
    delta_arg(         "D", "delta"        , "minimum change of data likelihood"            ,false,     0.001,  "double")
{
    try {
	TCLAP::CmdLine cmd("This program is BatchWorker. It collects data.", ' ', "");

        cmd.add(delta_arg);
        cmd.add(incF_arg);
        cmd.add(pruningOff_arg);
        cmd.add(l1_arg);
        cmd.add(tree_arg);
        cmd.add(discount_arg);
        cmd.add(repeat_arg);
        cmd.add(eval_arg);
        cmd.add(incT_arg);
        cmd.add(maxT_arg);
        cmd.add(minT_arg);
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
    // check mode
    bool mode_ok = false;
    for(string s : mode_vector) {
        if(mode_arg.getValue()==s) {
            mode = mode_arg.getValue();
            mode_ok = true;
            break;
        }
    }
    if(!mode_ok) {
        DEBUG_OUT(1,"Argument value for '" << mode_arg.getName() << "' must be one of:");
        for(string s : mode_vector) {
            DEBUG_OUT(1,"    " << s);
        }
    } else {
        if(mode!="DRY" && mode!="RANDOM" && mode!="CRF" && mode!="MODEL_BASED_UTREE" && mode!="VALUE_BASED_UTREE") {
            DEBUG_OUT(1,"mode '" << mode << "' currently not supported");
            mode_ok = false;
        }
    }

    // check number of training samples
    bool samples_ok = true;
    if(minT_arg.getValue()<=0) {
        DEBUG_OUT(0,"Argument '" << minT_arg.getName() << "' requires a value grater than zero");
        samples_ok = false;
    } else {
        minT = minT_arg.getValue();
    }
    if(maxT_arg.getValue()!=-1 && maxT_arg.getValue()<minT) {
        DEBUG_OUT(0,"Argument '" << maxT_arg.getName() << "' requires a value greater that that of '" << minT_arg.getName() << "'");
        samples_ok = false;
    } else {
        maxT = maxT_arg.getValue();
        maxT = maxT==-1?minT:maxT;
    }
    if(incT_arg.getValue()!=-1 && incT_arg.getValue()<=0) {
        DEBUG_OUT(0,"Argument '" << incT_arg.getName() << "' requires a value greater that zero");
        samples_ok = false;
    } else {
        incT = incT_arg.getValue();
        incT = incT==-1?(maxT-minT):incT;
        incT = incT==0?1:incT;
    }

    // check evaluation length
    bool eval_ok = true;
    if(eval_arg.getValue()<=0) {
        DEBUG_OUT(0,"Argument '" << eval_arg.getName() << "' requires a value grater than zero");
        eval_ok = false;
    }

    return (mode_ok && samples_ok && eval_ok);
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
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(1)
#endif
    for(int repeat_index=1; repeat_index<=repeat_n; ++repeat_index) {
        for(int training_length=minT; training_length<=maxT; training_length+=incT) {
            DEBUG_OUT(2,"Training length: " << training_length);

            int this_episode_counter;
#ifdef USE_OMP
#pragma omp critical
#endif
            {
                this_episode_counter = episode_counter++;
            }

            // environment
            shared_ptr<Environment> environment;
            // learner
            shared_ptr<FeatureLearner> learner;
            // spaces
            action_ptr_t action_space;
            observation_ptr_t observation_space;
            reward_ptr_t reward_space;
            // current instance
            instance_t * current_instance = nullptr;
            // data
            double mean_reward = 0;  // for all
            double data_likelihood;  // for CRF
            int nr_features;         // for CRF
            int utree_size;          // for UTree
            double utree_score;      // for UTree

            // initialize environment and get spaces
            environment = make_shared<CheeseMaze>();
            environment->get_spaces(action_space,observation_space,reward_space);

            // initialize learner
            if(mode=="DRY" || mode=="RANDOM") {
                // no learner
            } else if(mode=="CRF") {
                learner = make_shared<KMarkovCRF>();
            } else if(mode=="MODEL_BASED_UTREE") {
                learner = make_shared<UTree>(discount_arg.getValue());
            } else if(mode=="VALUE_BASED_UTREE") {
                learner = make_shared<UTree>(discount_arg.getValue());
            } else {
                DEBUG_DEAD_LINE;
            }

            // collect data and train learner
            if(mode=="DRY" || mode=="RANDOM") {
                // no data to collect, nothing to learn
            } else if(mode=="CRF" || mode=="MODEL_BASED_UTREE" || mode=="VALUE_BASED_UTREE"){
                // get features and spaces
                learner->set_spaces(*environment);
                learner->set_features(*environment);
                // collect data
                auto observer = dynamic_pointer_cast<HistoryObserver>(learner);
                if(observer==nullptr) {
                    DEBUG_DEAD_LINE;
                } else {
                    collect_random_data(environment, observer, training_length, current_instance);
                }
                // train learner
                if(mode=="CRF") {
                    train_CRF(learner, data_likelihood, nr_features);
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

            // evaluate
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
            } else if(mode=="CRF" || mode=="MODEL_BASED_UTREE"){
                // cast to predictor
                auto pred = dynamic_pointer_cast<Predictor>(learner);
                if(pred==nullptr) {
                    DEBUG_DEAD_LINE;
                }
                // initialize planner
                LookAheadSearch planner(discount_arg.getValue());
                planner.set_spaces(action_space, observation_space, reward_space);
                // do planned steps
                for(int step_idx=0; step_idx<eval_arg.getValue(); ++step_idx) {
                    // do planning and select action
                    action_ptr_t action;
                    if(pruningOff_arg.getValue() || step_idx==0) {
                        planner.clear_tree();
                        planner.build_tree(current_instance, *pred, tree_arg.getValue());
                    } else {
                        planner.fully_expand_tree(*pred, tree_arg.getValue());
                    }
                    action = planner.get_optimal_action();
                    // actually perform the transition
                    observation_ptr_t observation;
                    reward_ptr_t reward;
                    environment->perform_transition(action, observation, reward);
                    mean_reward += reward->get_value();
                    current_instance = current_instance->append_instance(action, observation, reward);
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
                    current_instance = current_instance->append_instance(action, observation, reward);
                    DEBUG_OUT(2,"    " << action << "	" << observation << "	" << reward);
                }
            } else {
                DEBUG_DEAD_LINE;
            }

            // calculate mean reward
            mean_reward/=eval_arg.getValue();

            // write output
#ifdef USE_OMP
#pragma omp critical
#endif
            {
                if(mode=="CRF") {
                    LOG(this_episode_counter << "	" << training_length << "	" << eval_arg.getValue() << "	" << mean_reward << "	" << data_likelihood << "	" << nr_features << "	" << l1_arg.getValue());
                } else if(mode=="MODEL_BASED_UTREE" || mode=="VALUE_BASED_UTREE") {
                    LOG(this_episode_counter << "	" << training_length << "	" << eval_arg.getValue() << "	" << mean_reward << "	" << utree_score << "	" << utree_size);
                } else if(mode=="DRY" || mode=="RANDOM") {
                    LOG(this_episode_counter << "	" << training_length << "	" << eval_arg.getValue() << "	" << mean_reward);
                } else {
                    DEBUG_DEAD_LINE;
                }
            }

            delete current_instance;
        }
    } // end omp parallel for
}

void BatchWorker::collect_random_data(std::shared_ptr<Environment> env,
                                      std::shared_ptr<HistoryObserver> obs,
                                      const int& length,
                                      instance_t*& i) {
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
        if(i==nullptr) {
            i = instance_t::create(action, observation, reward);
        } else {
            i = i->append_instance(action, observation, reward);
        }
    }
}

void BatchWorker::train_CRF(std::shared_ptr<FeatureLearner> learner, double& likelihood, int& features) {
    // cast to correct type
    auto crf = dynamic_pointer_cast<KMarkovCRF>(learner);
    // check for cast failure
    if(crf==nullptr) {
        DEBUG_DEAD_LINE;
        return;
    }
    // optimize crf
    double old_likelihood = -DBL_MAX, new_likelihood = 0;
    while(new_likelihood-old_likelihood>delta_arg.getValue()) {
        old_likelihood = new_likelihood;
        crf->construct_candidate_features(1);
        crf->score_candidates_by_gradient();
        crf->add_candidate_features_to_active(incF_arg.getValue());
        crf->optimize_model(l1_arg.getValue(), 50, &new_likelihood);
        crf->erase_zero_features();
    }
    crf->optimize_model(0,100, &likelihood);
    // get number of features
    features = crf->get_number_of_features();
}

void BatchWorker::train_value_based_UTree(std::shared_ptr<FeatureLearner> learner, int& size, double& score) {
    // cast to correct type
    auto utree = dynamic_pointer_cast<UTree>(learner);
    // check for cast failure
    if(utree==nullptr) {
        DEBUG_DEAD_LINE;
        return;
    }
    // expand utree
    utree->set_expansion_type(UTree::UTILITY_EXPANSION);
    double score_threshold = 1e-5;
    score = DBL_MAX;
    while(score >= score_threshold) {
        score = utree->expand_leaf_node(score_threshold);
    }
    // get tree size
    size = utree->get_tree_size();
}

void BatchWorker::train_model_based_UTree(std::shared_ptr<FeatureLearner> learner, int& size, double& score) {
    // cast to correct type
    auto utree = dynamic_pointer_cast<UTree>(learner);
    // check for cast failure
    if(utree==nullptr) {
        DEBUG_DEAD_LINE;
        return;
    }
    // expand utree
    utree->set_expansion_type(UTree::OBSERVATION_REWARD_EXPANSION);
    double score_threshold = 1;
    score = DBL_MAX;
    while(score >= score_threshold ) {
        score = utree->expand_leaf_node(score_threshold);
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
    LOG_COMMENT("Training Length: " << minT << "--" << maxT << " (step size: " << incT << ")");
    LOG_COMMENT("Evaluation Length: " << eval_arg.getValue());
    LOG_COMMENT("Repetitions: " << repeat_arg.getValue());
    LOG_COMMENT("Discount: " << discount_arg.getValue());
    if(mode=="CRF" || mode=="MODEL_BASED_UTREE") {
        LOG_COMMENT("Max Look-Ahead Tree Size: " << tree_arg.getValue());
        LOG_COMMENT("Pruning: " << (pruningOff_arg.getValue()?"off":"on"));
    }
    if(mode=="CRF") {
        LOG_COMMENT("L1-factor: " << l1_arg.getValue());
        LOG_COMMENT("Feature Increment: " << incF_arg.getValue());
        LOG_COMMENT("Likelihood Delta: " << delta_arg.getValue());
    }

    LOG("");

    if(mode=="CRF") {
        LOG_COMMENT("Episode	training_length	evaluation_length	mean_reward	data_likelihood	nr_features	l1_factor");
    } else if(mode=="MODEL_BASED_UTREE" || mode=="VALUE_BASED_UTREE") {
        LOG_COMMENT("Episode	training_length	evaluation_length	mean_reward	utree_score	utree_size");
    } else if(mode=="DRY" || mode=="RANDOM") {
        LOG_COMMENT("Episode	training_length	evaluation_length	mean_reward");
    } else {
        DEBUG_DEAD_LINE;
    }

    LOG("");
}
