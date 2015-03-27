#include <iostream>
#include <time.h>
#include <vector>
#include <set>
#include <tuple>
#include <unistd.h>
#include <memory>

#include <tclap/CmdLine.h>

#include <util/util.h>
#include <util/QtUtil.h>

#include "SearchTree.h"
#include "MonteCarloTreeSearch.h"
#include "TightRope.h"
#include "DynamicTightRope.h"
#include "UnitTestEnvironment.h"
#include "GamblingHall.h"

#include <util/return_tuple.h>
#include <util/pretty_printer.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::cout;
using std::endl;
using std::shared_ptr;
using util::Range;

using namespace tree_policy;
using namespace value_heuristic;
using namespace backup_method;

static const std::set<std::string> mode_set = {"SAMPLE",
                                               "WATCH",
                                               "EVAL"};
static const std::set<std::string> environment_set = {"TightRope",
                                                      "DynamicTightRope",
                                                      "GamblingHall",
                                                      "UnitTest"};
static const std::set<std::string> accumulate_set = {"min",
                                                     "mean",
                                                     "max"};
static const std::set<std::string> graph_type_set = {"TREE",
                                                     "PARTIAL_DAG",
                                                     "FULL_DAG"};
static const std::set<std::string> backup_type_set = {"BACKUP_TRACE",
                                                      "BACKUP_ALL"};
static const std::set<std::string> backup_method_set = {"Bellman",
                                                        "BellmanTreePolicy",
                                                        "MonteCarlo"};
static const std::set<std::string> value_heuristic_set = {"Zero",
                                                          "Rollout"};
static const std::set<std::string> tree_policy_set = {"UCB1",
                                                      "UCB_Plus",
                                                      "Uniform"};

// the command line arguments
static TCLAP::ValueArg<std::string> mode_arg(        "m", "mode",\
                                                     "Mode to use "+util::container_to_str(mode_set,", ","(",")")+".\n'SAMPLE' takes <n> (--sample_n <n>) samples for different state-action \
pairs from the environment <e> (--environment <e>) and prints the data to \
std::cout.\n'WATCH' runs tree search with <n> rollouts per step and performs <s> \
(--step_n <s>) steps. The progress during tree building can be watched at a \
different level of detail (--progress).\n'EVAL' performs <r> (--run_n <r>) \
runs with a series of trials for different numbers of rollouts <n>, <n>+<i>, \
... , <m> (--sample_incr <i> --sample_max <m>) and <s> steps per trial and \
prints the mean reward per trial to std::cout.",\
                                                     true, "SAMPLE", "string");
static TCLAP::ValueArg<std::string> environment_arg( "e", "environment",\
                                                     "Environment to use "+util::container_to_str(environment_set,", ","(",")")+".\nDynamicTightRope: The agent has integer position and velocity. SAMPLE mode \
prints the max-reward (over actions) for a given position and \
velocity.\nTightRope: The agent has integer position. SAMPLE mode prints the \
reward for a given position and action.", \
                                                     true, "DynamicTightRope", "string");
static TCLAP::ValueArg<int> sample_n_arg(            "n", "sample_n",\
                                                     "Number of samples/rollouts.",\
                                                     false, 1, "int" );
static TCLAP::ValueArg<int> sample_incr_arg(         "i", "sample_incr",\
                                                     "Increment of the number rollouts. ",\
                                                     false, 1, "int" );
static TCLAP::ValueArg<int> sample_max_arg(          "x", "sample_max",\
                                                     "Maximum number of rollouts.",\
                                                     false, 100, "int" );
static TCLAP::ValueArg<int> step_n_arg(              "s", "step_n",\
                                                     "Number of steps to perform (0 for infinite / until terminal state).",\
                                                     false, 0, "int" );
static TCLAP::ValueArg<int> run_n_arg(               "r", "run_n", \
                                                     "Number of runs to perform for evaluations.", \
                                                     false, 1, "int" );
static TCLAP::ValueArg<int> watch_progress_arg(      "p", "progress",\
                                                     "Level of detail for watching progress (0,...,3).",\
                                                     false, 1, "int");
static TCLAP::SwitchArg no_graphics_arg(             "g", "no_graphics",\
                                                     "Don't generate graphics."\
                                                     , false);
static TCLAP::ValueArg<std::string> accumulate_arg(  "a", "accumulate", \
                                                     "How to accumulate values "+util::container_to_str(accumulate_set,", ","(",")")+"."\
                                                     , false, "mean", "string");
static TCLAP::ValueArg<std::string> graph_type_arg(  "", "graph_type", \
                                                     "Type of the graph to use: "+util::container_to_str(graph_type_set,", ","(",")")+"." \
                                                     , false, "FULL_DAG", "string");
static TCLAP::ValueArg<std::string> backup_type_arg( "", "backup_type",      \
                                                     "Type of backups to do: "+util::container_to_str(backup_type_set,", ","(",")")+"." \
                                                     , false, "BACKUP_ALL", "string");
static TCLAP::ValueArg<std::string> backup_method_arg( "", "backup_method", \
                                                     "Method to use for backups: "+util::container_to_str(backup_method_set,", ","(",")")+"." \
                                                     , false, "Bellman", "string");
static TCLAP::ValueArg<std::string> value_heuristic_arg( "", "value_heuristic", \
                                                       "Method to use for initializing leaf-node values: "+util::container_to_str(value_heuristic_set,", ","(",")")+"." \
                                                       , false, "Zero", "string");
static TCLAP::ValueArg<std::string> tree_policy_arg( "", "tree_policy",      \
                                                     "What tree policy to use "+util::container_to_str(tree_policy_set,", ","(",")")+"." \
                                                     , false, "UCB1", "string");
static TCLAP::ValueArg<double> discount_arg(         "d", "discount", "Discount for the returns"
                                                     , false, 0.9, "double");
static TCLAP::ValueArg<double> exploration_arg(      "", "exploration", "Weigh for exploration term in upper bound policies."
                                                     , false, 0.707, "double");
static TCLAP::ValueArg<double> rollout_length_arg(   "", "rollout_length", "Length of rollouts from leaf nodes. Use negative values for rollouts to \
terminal state if there is one and one-step if there is no terminal state."
                                                     , false, -1, "double");
static TCLAP::ValueArg<int> random_seed_arg(         "", "random_seed", \
                                                     "Random seed to use. For negative values (default) current time is used as seed.", \
                                                     false, -1, "int" );
static TCLAP::SwitchArg no_header_arg(               "", "no_header",\
                                                     "In EVAL mode (-m EVAL) don't print a head line containing the column names."\
                                                     , false);

bool check_arguments();
SearchTree::GRAPH_TYPE get_graph_type();
MonteCarloTreeSearch::BACKUP_TYPE get_backup_type();

int main(int argn, char ** args) {

    // get command line arguments
    try {
	TCLAP::CmdLine cmd("Sample an evironment or perform online search", ' ', "");
        cmd.add(no_header_arg);
        cmd.add(random_seed_arg);
        cmd.add(accumulate_arg);
        cmd.add(tree_policy_arg);
        cmd.add(value_heuristic_arg);
        cmd.add(backup_method_arg);
        cmd.add(backup_type_arg);
        cmd.add(graph_type_arg);
        cmd.add(no_graphics_arg);
        cmd.add(watch_progress_arg);
        cmd.add(rollout_length_arg);
        cmd.add(exploration_arg);
        cmd.add(discount_arg);
        cmd.add(run_n_arg);
        cmd.add(step_n_arg);
        cmd.add(sample_max_arg);
        cmd.add(sample_incr_arg);
        cmd.add(sample_n_arg);
        cmd.add(environment_arg);
        cmd.add(mode_arg);

	// parse args (throws execption in case of failure)
	cmd.parse(argn, args);

        if(!check_arguments()) return 0;

    } catch (TCLAP::ArgException &e) {
        // catch any exceptions
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }

    // set random seeds
    if(random_seed_arg.getValue()<0) {
        auto seed = time(nullptr);
        srand(seed);
        srand48(seed);
        if(mode_arg.getValue()!="EVAL") {
            cout << "Using seed: " << seed << endl;
        }
    } else {
        auto seed = random_seed_arg.getValue();
        srand(seed);
        srand48(seed);
        if(mode_arg.getValue()!="EVAL") {
            cout << "Reusing seed: " << seed << endl;
        }
    }

    // set up environment
    std::shared_ptr<Environment> environment;
    if(environment_arg.getValue()=="TightRope") {
        environment.reset(new TightRope(15));
    } else if(environment_arg.getValue()=="DynamicTightRope") {
        environment.reset(new DynamicTightRope(50, 10));
    } else if(environment_arg.getValue()=="GamblingHall") {
        environment.reset(new GamblingHall(10, 1));
    } else if(environment_arg.getValue()=="UnitTest") {
        environment.reset(new UnitTestEnvironment());
    } else {
        cout << "Unknown environment" << endl;
        DEBUG_DEAD_LINE;
    }
    DEBUG_OUT(1, "States: " << environment->get_states());
    DEBUG_OUT(1, "Actions: " << environment->get_actions());
    typedef Environment::state_t state_t;
    typedef Environment::action_t action_t;
    typedef Environment::reward_t reward_t;

    // set up search tree
    shared_ptr<SearchTree>      search_tree;
    shared_ptr<TreePolicy>      tree_policy;
    shared_ptr<ValueHeuristic>  value_heuristic;
    shared_ptr<BackupMethod>    backup_method;
    // set tree policy
    if(tree_policy_arg.getValue()=="UCB1") {
        tree_policy.reset(new UCB1(exploration_arg.getValue()));
    } else if(tree_policy_arg.getValue()=="UCB_Plus") {
        tree_policy.reset(new UCB_Plus(exploration_arg.getValue()));
    } else if(tree_policy_arg.getValue()=="Uniform") {
        tree_policy.reset(new Uniform());
    } else DEBUG_DEAD_LINE;
    // set value heuristic
    if(value_heuristic_arg.getValue()=="Zero") {
        value_heuristic.reset(new Zero());
    } else if(value_heuristic_arg.getValue()=="Rollout") {
        value_heuristic.reset(new Rollout(rollout_length_arg.getValue()));
    } else DEBUG_DEAD_LINE;
    // set backup method
    if(backup_method_arg.getValue()=="Bellman") {
        backup_method.reset(new Bellman());
    } else if(backup_method_arg.getValue()=="BellmanTreePolicy") {
        backup_method.reset(new Bellman(tree_policy));
    } else if(backup_method_arg.getValue()=="MonteCarlo") {
        backup_method.reset(new MonteCarlo());
    } else DEBUG_DEAD_LINE;
    // set search tree
    state_t root_state = environment->default_state();
    search_tree.reset(new MonteCarloTreeSearch(root_state,
                                               environment,
                                               discount_arg.getValue(),
                                               get_graph_type(),
                                               tree_policy,
                                               value_heuristic,
                                               backup_method,
                                               get_backup_type()));

    // different modes
    if(mode_arg.getValue()=="SAMPLE") {
        if(environment_arg.getValue()!="DynamicTightRope") {
            cout << "run,state,action,reward" << endl;
            for(int run : Range(sample_n_arg.getValue())) {
                for(auto state_from : environment->get_states()) {
                    if(environment->is_terminal_state(state_from)) continue;
                    for(auto action : environment->get_actions()) {
                        RETURN_TUPLE(state_t,state_to,reward_t,reward) = environment->sample(state_from, action);
                        cout << QString("%1,%2,%3,%4").arg(run).arg(state_from).arg(action).arg(reward) << endl;
                    }
                }
            }
        } else {
            cout << "run,position,velocity,"+accumulate_arg.getValue()+"_reward" << endl;
            for(int run : Range(sample_n_arg.getValue())) {
                for(auto state_from : environment->get_states()) {
                    double accum_reward;
                    auto env = std::dynamic_pointer_cast<DynamicTightRope>(environment);
                    DEBUG_EXPECT(0,env!=nullptr);
                    RETURN_TUPLE(int,position,int,velocity) = env->get_position_and_velocity(state_from);
                    if(accumulate_arg.getValue()=="min") {
                        accum_reward = DBL_MAX;
                    } else if(accumulate_arg.getValue()=="max") {
                        accum_reward = -DBL_MAX;
                    } else if(accumulate_arg.getValue()=="mean") {
                        accum_reward = 0;
                    } else {
                        DEBUG_DEAD_LINE;
                    }
                    for(auto action : environment->get_actions()) {
                        RETURN_TUPLE(state_t,state_to,reward_t,reward) = environment->sample(state_from, action);
                        if(accumulate_arg.getValue()=="min") {
                            accum_reward = std::min(reward,accum_reward);
                        } else if(accumulate_arg.getValue()=="max") {
                            accum_reward = std::max(reward,accum_reward);
                        } else if(accumulate_arg.getValue()=="mean") {
                            accum_reward += reward/environment->get_actions().size();
                        } else {
                            DEBUG_DEAD_LINE;
                        }
                    }
                    cout << QString("%1,%2,%3,%4").arg(run).arg(position).arg(velocity).arg(accum_reward) << endl;
                }
            }
        }
    } else if(mode_arg.getValue()=="WATCH") {
        cout << "Watch progress..." << endl;
        for(int step : Range(0,step_n_arg.getValue())) {
            if(environment->is_terminal_state(root_state)) break;
            for(int sample : Range(sample_n_arg.getValue())) {
                search_tree->next();
                if(watch_progress_arg.getValue()>=3) {
                    if(!no_graphics_arg.getValue()) {
                        search_tree->toPdf("tree.pdf");
                    }
                    cout << "Sample # " << sample+1 << endl;
                    getchar();
                }
            }
            if(step<step_n_arg.getValue()) { // don't prune in last step
                auto action = search_tree->recommend_action();
                auto state_reward = environment->sample(root_state,action);
                auto state = std::get<0>(state_reward);
                auto reward = std::get<1>(state_reward);
                if(watch_progress_arg.getValue()>=2 && !no_graphics_arg.getValue()) {
                    search_tree->toPdf("tree.pdf");
                    getchar();
                }
                search_tree->prune(action,state);
                if(watch_progress_arg.getValue()>=2) {
                    if(!no_graphics_arg.getValue()) {
                        search_tree->toPdf("tree.pdf");
                    }
                    cout << "Step # " << step+1 <<
                        ": (action --> state, reward) = (" <<
                        environment->action_name(action) << " --> " <<
                        environment->state_name(state) << ", " <<
                        reward << ")" << endl;
                    getchar();
                }
                root_state = state;
            }
        }
        if(watch_progress_arg.getValue()>=1 && !no_graphics_arg.getValue()) {
            search_tree->toPdf("tree.pdf");
        }
    } else if(mode_arg.getValue()=="EVAL") {
        // print header
        if(!no_header_arg.getValue()) {
            cout << "mean reward,number of roll-outs,run,method" << endl;
        }
        // string describing method
        QString tree_policy_string = tree_policy_arg.getValue().c_str();
        if(tree_policy_string=="UCB1" || tree_policy_string=="UCB_Plus") {
            tree_policy_string += QString("(%1)").arg(exploration_arg.getValue());
        }
        QString value_heuristic_string = value_heuristic_arg.getValue().c_str();
        if(value_heuristic_string=="Rollout") {
            value_heuristic_string += QString("(%1)").arg(rollout_length_arg.getValue());
        }
        QString method_string = QString("%1 / %2 / %3 [%4 / %5]").
            arg(tree_policy_string).
            arg(value_heuristic_string).
            arg(backup_method_arg.getValue().c_str()).
            arg(graph_type_arg.getValue().c_str()).
            arg(backup_type_arg.getValue().c_str());
        // several runs
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(2)
#endif
        for(int run : Range(run_n_arg.getValue())) {
            DEBUG_OUT(1, "Run # " << run);
            // with one trial for a different number of samples
            for(int sample_n : Range(sample_n_arg.getValue(), sample_max_arg.getValue(), sample_incr_arg.getValue())) {
                DEBUG_OUT(1, "Samples: " << sample_n);
                double reward_sum = 0;
                // for each number of samples a number of steps (or until
                // terminal state) is performed
                int step = 1;
                // reinitialize tree to default state
                root_state = environment->default_state();
                search_tree->init(root_state);
                while(true) {
                    // build tree
                    repeat(sample_n) {
                        search_tree->next();
                    }
                    // perform step
                    auto action = search_tree->recommend_action();
                    auto state_reward = environment->sample(root_state,action);
                    auto state = std::get<0>(state_reward);
                    reward_sum += std::get<1>(state_reward);
                    // break on terminal state
                    if(environment->has_terminal_state() && environment->is_terminal_state(state)) break;
                    // break if (maximum) number of steps was set and reached
                    if(step_n_arg.getValue()>0 && step>=step_n_arg.getValue()) break;
                    // otherwise prune tree and increment step number
                    search_tree->prune(action,state);
                    root_state = state;
                    ++step;
                }
#ifdef USE_OMP
#pragma omp critical (BatchWorker)
#endif
                cout << QString("%1,%2,%3,%4").
                    arg(reward_sum/step).
                    arg(sample_n).
                    arg(run).
                    arg(method_string) << endl;
            }
        }
    } else {
        DEBUG_ERROR("Unexpected mode");
    }
}

bool check_arguments() {
    bool ok = true;
    // check mode
    if(mode_set.find(mode_arg.getValue())==mode_set.end()) {
        ok = false;
        cout << "Mode must be one of:" << util::container_to_str(mode_set,"\n\t","\n\t") << endl;
    }
    // check environment
    if(environment_set.find(environment_arg.getValue())==environment_set.end()) {
        ok = false;
        cout << "Environment must be one of:" << util::container_to_str(environment_set,"\n\t","\n\t") << endl;
    }
    // check sample number
    if(sample_n_arg.getValue()<0) {
        ok = false;
        cout << "Number of samples must be non-negative." << endl;
    }
    // check accumulation
    if(accumulate_set.find(accumulate_arg.getValue())==accumulate_set.end()) {
        ok = false;
        cout << "Accumulation must be one of:" << util::container_to_str(accumulate_set,"\n\t","\n\t") << endl;
    }
    // check graph type
    if(graph_type_set.find(graph_type_arg.getValue())==graph_type_set.end()) {
        ok = false;
        cout << "Graph type must be one of:" << util::container_to_str(graph_type_set,"\n\t","\n\t") << endl;
    }
    // check backup type
    if(backup_type_set.find(backup_type_arg.getValue())==backup_type_set.end()) {
        ok = false;
        cout << "Backup type must be one of:" << util::container_to_str(backup_type_set,"\n\t","\n\t") << endl;
    }
    // check backup method
    if(backup_method_set.find(backup_method_arg.getValue())==backup_method_set.end()) {
        ok = false;
        cout << "Backup method must be one of:" << util::container_to_str(backup_method_set,"\n\t","\n\t") << endl;
    }
    // check value heuristic
    if(value_heuristic_set.find(value_heuristic_arg.getValue())==value_heuristic_set.end()) {
        ok = false;
        cout << "value heuristic must be one of:" << util::container_to_str(value_heuristic_set,"\n\t","\n\t") << endl;
    }
    // check tree policy
    if(tree_policy_set.find(tree_policy_arg.getValue())==tree_policy_set.end()) {
        ok = false;
        cout << "Tree policy must be one of:" << util::container_to_str(tree_policy_set,"\n\t","\n\t") << endl;
    }
    if(discount_arg.getValue()<0 || discount_arg.getValue()>1) {
        ok = false;
        cout << "Discount must be in [0:1]" << endl;
    }
    if(exploration_arg.getValue()<0) {
        ok = false;
        cout << "Exploration must be positive" << endl;
    }
    return ok;
}

SearchTree::GRAPH_TYPE get_graph_type() {
    if(graph_type_arg.getValue()=="TREE") return SearchTree::TREE;
    if(graph_type_arg.getValue()=="PARTIAL_DAG") return SearchTree::PARTIAL_DAG;
    if(graph_type_arg.getValue()=="FULL_DAG") return SearchTree::FULL_DAG;
    DEBUG_DEAD_LINE;
    return SearchTree::TREE;
}

MonteCarloTreeSearch::BACKUP_TYPE get_backup_type() {
    if(backup_type_arg.getValue()=="BACKUP_TRACE") return MonteCarloTreeSearch::BACKUP_TRACE;
    if(backup_type_arg.getValue()=="BACKUP_ALL") return MonteCarloTreeSearch::BACKUP_ALL;
    DEBUG_DEAD_LINE;
    return MonteCarloTreeSearch::BACKUP_TRACE;
}
