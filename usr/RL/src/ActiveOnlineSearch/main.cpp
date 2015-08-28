#include <iostream>
#include <time.h>
#include <vector>
#include <set>
#include <tuple>
#include <unistd.h>
#include <memory>

#include <tclap/CmdLine.h>

#include <util/util.h>
#include <util/return_tuple.h>
#include <util/Commander.h>

#include <QDateTime>
#include <util/QtUtil.h>

#include "TreeSearch/SearchTree.h"
#include "TreeSearch/MonteCarloTreeSearch.h"
#include "TreeSearch/ActiveTreeSearch.h"
#include "TreeSearch/RandomSearch.h"
#include "TreeSearch/NodeFinder.h"
#include "TreeSearch/TreePolicy.h"
#include "TreeSearch/ValueHeuristic.h"
#include "TreeSearch/BackupMethod.h"
#include "Environment/SimpleEnvironment.h"
#include "Environment/GamblingHall.h"
#include "Environment/BottleneckEnvironment.h"
#include "Environment/VarianceEnvironments.h"
#include "Environment/MC_versus_DP.h"
#include "Environment/Stochastic1D.h"
#include "Environment/NastyStochastic1D.h"
#include "../../../../share/src/FOL/fol_mcts_world.h"
#include "../../../../share/src/POMCP/mcts.h"

#include <omp.h>
#define USE_OMP

#include <util/return_tuple.h>
#include <util/pretty_printer.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::tuple;
using std::make_tuple;
using std::cout;
using std::cin;
using std::endl;
using std::shared_ptr;
using std::make_shared;
using util::Range;
using std::dynamic_pointer_cast;

using namespace node_finder;
using namespace tree_policy;
using namespace value_heuristic;
using namespace backup_method;

typedef Commander::ReturnType Ret;
typedef AbstractEnvironment::action_handle_t      action_handle_t;
typedef AbstractEnvironment::observation_handle_t observation_handle_t;
typedef AbstractEnvironment::reward_t             reward_t;

static QString graph_plot_file_name = "tree.pdf";
static QString graph_plot_command = "dot";
static QString graph_plot_parameters = "-Tpdf";
static bool write_log = false;
static std::ofstream log_file;
static Commander::CommandCenter commander;

static const std::set<std::string> mode_set = {"WATCH",
                                               "EVAL_ONLINE",
                                               "EVAL_ROOT_ACTION",
                                               "PLAY"};
static const std::set<std::string> environment_set = {"GamblingHall",
                                                      "FOL",
                                                      "SimpleEnvironment",
                                                      "BottleneckEnvironment",
                                                      "DLVSOR",
                                                      "LVSOR",
                                                      "MCVSDP",
                                                      "Stochastic1D",
                                                      "NastyStochastic1D"
};
static const std::set<std::string> graph_type_set = {"PlainTree",
                                                     "FullDAG",
                                                     "FullGraph",
                                                     "ObservationTree"};
static const std::set<std::string> backup_type_set = {"TRACE",
                                                      "PROPAGATE"};
static const std::set<std::string> rollout_storage_set = {"NONE",
                                                          "CONDENSED",
                                                          "FULL"};
static const std::set<std::string> backup_method_set = {"Bellman",
                                                        "MonteCarlo",
                                                        "HybridMCDP"};
static const std::set<std::string> value_heuristic_set = {"Zero",
                                                          "RolloutStatistics"};
static const std::set<std::string> tree_policy_set = {"UCB1",
                                                      "UCB_Variance",
                                                      "Uniform",
                                                      "HardUpper",
                                                      "Optimal",
                                                      "Quantile"};
static const std::set<std::string> graphics_type_set = {"svg",
                                                        "png",
                                                        "pdf"};

// the command line arguments
static TCLAP::ValueArg<std::string> mode_arg(        "m", "mode",\
                                                     "Mode to use "+util::container_to_str(mode_set,", ","(",")")+".\n'WATCH' runs tree search with <n> rollouts per step and performs <s> \
(--step_n <s>) steps. The progress during tree building can be watched at a \
different level of detail (--progress).\n'EVAL' performs <r> (--run_n <r>) \
runs with a series of trials for different numbers of rollouts <n>, <n>+<i>, \
... , <x> (--sample_incr <i> --sample_max <x>) and <s> steps per trial and \
prints the mean reward per trial to std::cout.",\
                                                     true, "WATCH", "string");
static TCLAP::ValueArg<std::string> environment_arg( "e", "environment",\
                                                     "Environment to use "+util::container_to_str(environment_set,", ","(",")")+".\nDynamicTightRope: The agent has integer position and velocity. SAMPLE mode \
prints the max-reward (over actions) for a given position and \
velocity.\nTightRope: The agent has integer position. SAMPLE mode prints the \
reward for a given position and action.", \
                                                     true, "DynamicTightRope", "string");
static TCLAP::ValueArg<int> sample_n_arg(            "n", "sample_n",\
                                                     "(default: 1) Number of samples/rollouts.",\
                                                     false, 1, "int" );
static TCLAP::ValueArg<int> sample_incr_arg(         "i", "sample_incr",\
                                                     "(default: 1) Increment of the number rollouts. ",\
                                                     false, 1, "int" );
static TCLAP::ValueArg<int> sample_max_arg(          "x", "sample_max",\
                                                     "(default: -1) Maximum number of rollouts (neative values for same as minimal).",\
                                                     false, -1, "int" );
static TCLAP::ValueArg<int> step_n_arg(              "s", "step_n",\
                                                     "(default: -1) Number of steps to perform (negative for infinite / until terminal state).",\
                                                     false, -1, "int" );
static TCLAP::ValueArg<int> run_n_arg(               "r", "run_n", \
                                                     "(default: 1) Number of runs to perform for evaluations.", \
                                                     false, 1, "int" );
static TCLAP::ValueArg<int> run_start_arg(               "", "run_start", \
                                                     "(default: 0) Index of first run (use this when adding more runs to a data set to avoid duplicate run indices).", \
                                                     false, 0, "int" );
static TCLAP::ValueArg<int> watch_progress_arg(      "p", "progress",\
                                                     "(default: 1) Level of detail for watching progress (0,...,3).",\
                                                     false, 1, "int");
static TCLAP::SwitchArg graphics_arg(                "g", "graphics",\
                                                     "(default: false) If true automatically generate graphics."\
                                                     , false);
static TCLAP::ValueArg<std::string> graph_type_arg(  "", "graph_type", \
                                                     "(default: PlainTree) Type of the graph to use: "+util::container_to_str(graph_type_set,", ","(",")")+"." \
                                                     , false, "PlainTree", "string");
static TCLAP::ValueArg<std::string> backup_type_arg( "", "backup_type",      \
                                                     "(default: PROPAGATE) Type of backups to do: "+util::container_to_str(backup_type_set,", ","(",")")+"." \
                                                     , false, "PROPAGATE", "string");
static TCLAP::ValueArg<std::string> rollout_storage_arg( "", "rollout_storage", \
                                                         "(default: NONE) Type of backups to do: "+util::container_to_str(rollout_storage_set,", ","(",")")+"." \
                                                         , false, "NONE", "string");
static TCLAP::ValueArg<std::string> backup_method_arg( "", "backup_method", \
                                                     "(default: Bellman) Method to use for backups: "+util::container_to_str(backup_method_set,", ","(",")")+"." \
                                                     , false, "Bellman", "string");
static TCLAP::ValueArg<std::string> value_heuristic_arg( "", "value_heuristic", \
                                                       "(default: RolloutStatistics) Method to use for initializing leaf-node values: "+util::container_to_str(value_heuristic_set,", ","(",")")+"." \
                                                       , false, "RolloutStatistics", "string");
static TCLAP::ValueArg<std::string> tree_policy_arg( "", "tree_policy",      \
                                                     "(default: UCB1) What tree policy to use "+util::container_to_str(tree_policy_set,", ","(",")")+"." \
                                                     , false, "UCB1", "string");
static TCLAP::ValueArg<std::string> action_policy_arg( "", "action_policy",      \
                                                     "(default: Optimal) Policy to choose action to actually perfrom "+util::container_to_str(tree_policy_set,", ","(",")")+"." \
                                                     , false, "Optimal", "string");
static TCLAP::ValueArg<std::string> backup_policy_arg( "", "backup_policy",      \
                                                     "(default: Optimal) Policy used with Bellman backups "+util::container_to_str(tree_policy_set,", ","(",")")+"." \
                                                     , false, "Optimal", "string");
static TCLAP::ValueArg<std::string> graphics_type_arg( "", "graphics_type",      \
                                                     "(default: pdf) Format for printing search tree "+util::container_to_str(graphics_type_set,", ","(",")")+"." \
                                                     , false, "pdf", "string");
static TCLAP::ValueArg<double> discount_arg(         "d", "discount", "(default: 1) Discount for the returns"
                                                     , false, 1, "double");
static TCLAP::ValueArg<double> T_action_arg(         "", "T_action", "(default: 0) Temperature for soft-max in action-policy"
                                                     , false, 0, "double");
static TCLAP::ValueArg<double> T_tree_arg(           "", "T_tree", "(default: 0) Temperature for soft-max in tree-policy"
                                                     , false, 0, "double");
static TCLAP::ValueArg<double> T_backup_arg(         "", "T_backup", "(default: 0) Temperature for soft-max in backup-policy for Bellman backups"
                                                     , false, 0, "double");
static TCLAP::ValueArg<double> prior_counts_arg(     "", "prior_counts", "(default: 0) Prior counts to use (negative values for auto)."
                                                     , false, 0, "double");
static TCLAP::ValueArg<double> exploration_arg(      "", "exploration", "(default: 0.707) Weigh for exploration term in upper bound policies."
                                                     , false, 0.707, "double");
static TCLAP::ValueArg<double> rollout_length_arg(   "", "rollout_length", "(default: -1) Length of rollouts from leaf nodes. Use negative values for rollouts to \
terminal state if there is one and one-step if there is no terminal state."
                                                     , false, -1, "double");
static TCLAP::ValueArg<int> random_seed_arg(         "", "random_seed", \
                                                     "(default: -1) Random seed to use. For negative values (default) current time is used as seed.", \
                                                     false, -1, "int" );
static TCLAP::SwitchArg no_header_arg(               "", "no_header",\
                                                     "(default: false) In EVAL mode (-m EVAL) don't print a head line containing the column names."\
                                                     , false);
static TCLAP::SwitchArg prune_all_arg(               "", "prune_all",\
                                                     "(default: false) When 'true' prune all branches when performing a transition instead of reusing sub-branch.."\
                                                     , false);
static TCLAP::ValueArg<int> threads_arg(             "t", "threads", \
                                                     "(default: 0) Maximum number of threads to use by calling omp_set_num_threads(). A value of \
Zero (default) or below does not restict the number of threads so the \
OMP_NUM_THREADS environment variable will take effect (if set).", \
                                                     false, 0, "int" );
static TCLAP::SwitchArg active_arg(                  "", "active",\
                                                     "(default: false) Use active tree search."\
                                                     , false);
static TCLAP::SwitchArg random_arg(                  "", "random",\
                                                     "(default: false) Use random tree search."\
                                                     , false);
static TCLAP::ValueArg<std::string> traces_arg(      "", "traces", \
                                                     "(default: empty string) For non-empty string, write the action, observation, \
and (if possible) state description to the given file."\
                                                     , false, "", "string");
static TCLAP::ValueArg<double> quantile_arg(         "q", "quantile", "(default: 0.75) Quantile to use for Quantile tree-policy."
                                                     , false, 0.75, "double");

void plot_graph(shared_ptr<AbstractSearchTree>  search_tree, QString file_name = "") {
    if(file_name=="") {
        file_name = graph_plot_file_name;
    }
    search_tree->plot_graph(file_name.toLatin1(),
                            graph_plot_command.toLatin1(),
                            graph_plot_parameters.toLatin1(),
                            false);
}

void write_state_to_log(int run, std::shared_ptr<AbstractEnvironment> environment) {
    auto interface_marc = dynamic_pointer_cast<InterfaceMarc>(environment);
    if(interface_marc!=nullptr) {
        auto fol_world = dynamic_pointer_cast<FOL_World>(interface_marc->env_marc);
        if(fol_world) {
            log_file << run << "	State = ";
            fol_world->write_current_state(log_file);
            log_file << endl;
        }
    }
}

void write_state_to_cout(std::shared_ptr<AbstractEnvironment> environment) {
    auto interface_marc = dynamic_pointer_cast<InterfaceMarc>(environment);
    if(interface_marc!=nullptr) {
        auto fol_world = dynamic_pointer_cast<FOL_World>(interface_marc->env_marc);
        if(fol_world) {
            cout << "	State = ";
            fol_world->write_current_state(cout);
            cout << endl;
        }
    }
}

void prompt_for_command() {
    bool ok = false;
    while(!ok) {
        cout << "Your command: ";
        std::string command;
        getline(cin,command);
        cout << commander.execute(command.c_str(),ok) << endl;
        if(command!="") ok = false;
    }
}

bool check_arguments();

shared_ptr<AbstractEnvironment> get_environment();

shared_ptr<NodeFinder> get_node_finder();

shared_ptr<TreePolicy> get_policy(int type); // 0: tree, 1: action, 2: backup

shared_ptr<ValueHeuristic> get_value_heuristic();

shared_ptr<BackupMethod> get_backup_method();

MonteCarloTreeSearch::BACKUP_TYPE get_backup_type();

MonteCarloTreeSearch::ROLLOUT_STORAGE get_rollout_storage();


tuple<shared_ptr<AbstractSearchTree>,
      shared_ptr<AbstractEnvironment>> setup();

QString header(int argn, char ** args);

int main(int argn, char ** args) {

    // get command line arguments
    try {
	TCLAP::CmdLine cmd("Sample an evironment or perform online search", ' ', "");
        cmd.add(quantile_arg);
        cmd.add(traces_arg);
        cmd.add(random_arg);
        cmd.add(active_arg);
        cmd.add(threads_arg);
        cmd.add(prune_all_arg);
        cmd.add(no_header_arg);
        cmd.add(random_seed_arg);
        cmd.add(action_policy_arg);
        cmd.add(backup_policy_arg);
        cmd.add(tree_policy_arg);
        cmd.add(value_heuristic_arg);
        cmd.add(backup_method_arg);
        cmd.add(rollout_storage_arg);
        cmd.add(backup_type_arg);
        cmd.add(graph_type_arg);
        cmd.add(graphics_type_arg);
        cmd.add(graphics_arg);
        cmd.add(watch_progress_arg);
        cmd.add(rollout_length_arg);
        cmd.add(exploration_arg);
        cmd.add(prior_counts_arg);
        cmd.add(T_backup_arg);
        cmd.add(T_tree_arg);
        cmd.add(T_action_arg);
        cmd.add(discount_arg);
        cmd.add(run_start_arg);
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

    // open log file if given
    if(traces_arg.getValue()!="") {
        write_log = true;
        log_file.open(traces_arg.getValue());
        log_file << "run	action	observation	reward" << endl;
    }

    // set random seeds
    if(random_seed_arg.getValue()<0) {
        auto seed = time(nullptr);
        srand(seed);
        srand48(seed);
        cout << "# using given random seed: " << seed << endl;
    } else {
        auto seed = random_seed_arg.getValue();
        srand(seed);
        srand48(seed);
        cout << "# random seed: " << seed << endl;
    }

    // different modes
    cout << header(argn,args) << endl;
    if(mode_arg.getValue()=="WATCH") {
        // set some default commands
        commander.add_command("", []()->Ret{return {true,"Continuing..."};}, "Continue execution (empty string / only whitespace)");
        commander.add_command({"help","h"}, []()->Ret{cout << "\n\n" << commander.get_help_string() << "\n";return{true,""};}, "Print help");
        // set up
        RETURN_TUPLE(shared_ptr<AbstractSearchTree>, search_tree,
                     shared_ptr<AbstractEnvironment>, environment) = setup();
        cout << "Watch progress..." << endl;
        cout << "Method: " << *search_tree << endl;
        for(int step=0; step<step_n_arg.getValue() || step_n_arg.getValue()<0; ++step) {
            environment->reset_state();
            if(environment->is_terminal_state()) break;
            for(int sample : Range(sample_n_arg.getValue())) {
                search_tree->next();
                if(watch_progress_arg.getValue()>=3) {
                    if(graphics_arg.getValue()) {
                        plot_graph(search_tree);
                    }
                    cout << "Sample # " << sample+1 << endl;
                    prompt_for_command();
                }
            }
            // make a transition and prune
            environment->reset_state();
            auto action = search_tree->recommend_action();
            RETURN_TUPLE(observation_handle_t, observation,
                         reward_t, reward) = environment->transition(action);
            environment->make_current_state_default();
            // write
            if(write_log) {
                log_file << 0 << "	" <<
                    *action << "	" <<
                    *observation << "	" <<
                    reward << endl;
                write_state_to_log(0,environment);
            }
            if(watch_progress_arg.getValue()>=2) {
                cout << "Step # " << step+1 <<
                    ": (action --> observation, reward) = (" <<
                    *action << " --> " <<
                    *observation << ", " <<
                    reward << ")" << endl;
                write_state_to_cout(environment);
                if(environment->is_terminal_state()) {
                    cout << "Terminal state!" << endl;
                }
                if(graphics_arg.getValue()) {
                    plot_graph(search_tree);
                }
                prompt_for_command();
            }
             // prune (but not in last step to be able to visualize final tree)
            if(step<step_n_arg.getValue() || step_n_arg.getValue()) {
                if(prune_all_arg.getValue()) {
                    search_tree->init();
                } else {
                    search_tree->update(action,observation);
                }
                if(watch_progress_arg.getValue()>=2) {
                    cout << "tree pruned" << endl;
                    if(graphics_arg.getValue()) {
                        plot_graph(search_tree);
                    }
                    prompt_for_command();
                }
            }
        }
        if(watch_progress_arg.getValue()>=1 && graphics_arg.getValue()) {
            plot_graph(search_tree);
        }
    } else if(mode_arg.getValue()=="EVAL_ONLINE") {
        // print header
        if(!no_header_arg.getValue()) {
            cout << "reward sum,number of roll-outs,run,method" << endl;
        }
        QString method_string = "";
        if(active_arg.getValue()) {
            method_string = "ACTIVE";
        } else if(random_arg.getValue()) {
            method_string = "RANDOM";
        }
        // limit number of threads if required
        if(threads_arg.getValue()>0) {
            omp_set_num_threads(threads_arg.getValue());
        }
        int run_n = run_n_arg.getValue();
        int run_start = run_start_arg.getValue();
        int sample_n = sample_n_arg.getValue();
        int sample_max = sample_max_arg.getValue();
        if(sample_max<0) sample_max = sample_n;
        int sample_incr = sample_incr_arg.getValue();
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(2)
#endif
        // several runs
        for(int run=run_start; run<run_start+run_n; ++run) {

            // with one trial for a different number of samples
            for(int sample=sample_n; sample<=sample_max; sample+=sample_incr) {

                DEBUG_OUT(1, "run # " << run << ", samples: " << sample);

                // set up
                RETURN_TUPLE(shared_ptr<AbstractSearchTree>, search_tree,
                             shared_ptr<AbstractEnvironment>, environment) = setup();

                double reward_sum = 0;
                // for each number of samples a number of steps (or until
                // terminal state) is performed
                int step = 1;
                // initialize tree
                search_tree->init();
                while(true) {
                    // build tree
                    repeat(sample) {
                        search_tree->next();
                    }
                    // perform step
                    environment->reset_state();
                    auto action = search_tree->recommend_action();
                    RETURN_TUPLE(observation_handle_t, observation,
                                 reward_t, reward) = environment->transition(action);
                    environment->make_current_state_default();
                    reward_sum += reward;
                    // write log
#ifdef USE_OMP
#pragma omp critical (MCTS)
#endif
                    if(write_log) {
                        log_file << run << "	" <<
                            *action << "	" <<
                            *observation << "	" <<
                            reward << endl;
                        write_state_to_log(run,environment);
                    }
                    // break on terminal state
                    if(environment->is_terminal_state()) break;
                    // break if (maximum) number of steps was set and reached
                    if(step_n_arg.getValue()>=0 && step>=step_n_arg.getValue()) break;
                    // otherwise prune tree and increment step number
                    if(prune_all_arg.getValue()) {
                        search_tree->init();
                    } else {
                        search_tree->update(action,observation);
                    }
                    ++step;
                }
#ifdef USE_OMP
#pragma omp critical (MCTS)
#endif
                {
                    cout << QString("%1,	%2,	%3,	").
                        arg(reward_sum).
                        arg(sample).
                        arg(run);
                    if(method_string=="") {
                        cout << *search_tree;
                    } else {
                        cout << method_string;
                    }
                    cout << endl;
                }
            }
        }
    } else if(mode_arg.getValue()=="EVAL_ROOT_ACTION") {
        // print header
        if(!no_header_arg.getValue()) {
            cout << "action,	number of roll-outs,	run,	method" << endl;
        }
        QString method_string = "";
        if(active_arg.getValue()) {
            method_string = "ACTIVE";
        } else if(random_arg.getValue()) {
            method_string = "RANDOM";
        }
        // limit number of threads if required
        if(threads_arg.getValue()>0) {
            omp_set_num_threads(threads_arg.getValue());
        }
        int run_n = run_n_arg.getValue();
        int run_start = run_start_arg.getValue();
        int sample_n = sample_n_arg.getValue();
        int sample_max = sample_max_arg.getValue();
        if(sample_max<0) sample_max = sample_n;
        int sample_incr = sample_incr_arg.getValue();
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(2)
#endif
        // several runs
        for(int run=run_start; run<run_start+run_n; ++run) {

            // with one trial for a different number of samples
            for(int sample=sample_n; sample<=sample_max; sample+=sample_incr) {

                DEBUG_OUT(1, "run # " << run << ", samples: " << sample);

                // set up
                RETURN_TUPLE(shared_ptr<AbstractSearchTree>, search_tree,
                             shared_ptr<AbstractEnvironment>, environment) = setup();
                // initialize tree
                search_tree->init();
                // build tree
                repeat(sample) {
                    search_tree->next();
                }
                // get action
                auto action = search_tree->recommend_action();
                // write log
#ifdef USE_OMP
#pragma omp critical (MCTS)
#endif
                {
                    cout << *action << QString(",	%2,	%3,	").
                        arg(sample).
                        arg(run);
                    if(method_string=="") {
                        cout << *search_tree;
                    } else {
                        cout << method_string;
                    }
                    cout << endl;
                }
            }
        }
    } else if(mode_arg.getValue()=="PLAY") {
        // set up
        RETURN_TUPLE(shared_ptr<AbstractSearchTree>, search_tree,
                     shared_ptr<AbstractEnvironment>, environment) = setup();
        cout << "Play the environment..." << endl;
        environment->reset_state();
        auto available_actions = environment->get_actions();
        int max_idx = 0;
        reward_t reward_sum = 0;
        commander.add_command("action help", []()->Ret{cout << "\n\n" << commander.get_help_string() << "\n";return{false,""};}, "Print help");
        commander.add_command({"action"}, [&](int idx)->Ret{
                if(idx>max_idx) {
                    return {false,QString("Index must be in [0,%1]").arg(max_idx)};
                } else {
                    auto action = available_actions[idx];
                    RETURN_TUPLE(observation_handle_t, observation,
                                 reward_t, reward) = environment->transition(action);
                    reward_sum += reward;
                    cout << "(action --> observation, reward) = (" <<
                        *action << " --> " <<
                        *observation << ", " <<
                        reward << ")" << endl;
                    write_state_to_cout(environment);
                    if(environment->is_terminal_state()) {
                        cout << "Terminal state!" << endl;
                    }
                    return {true,QString("Performed action %1").arg(idx)};
                }
            }, "Perform an action");
        for(int step=1; !environment->is_terminal_state(); ++step) {
            available_actions = environment->get_actions();
            cout << "Choose an action for step " << step << ":" << endl;
            for(int idx=0; idx<(int)available_actions.size(); ++idx) {
                cout << "    (" << idx << "): " << *(available_actions[idx]) << endl;
                max_idx = idx;
            }
            bool ok = false;
            while(!ok) {
                cout << "action ";
                std::string command;
                getline(cin,command);
                command = "action "+command;
                cout << commander.execute(command.c_str(),ok) << endl;
                cout << "reward: sum=" << reward_sum << ", mean=" << reward_sum/step << endl;
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
        cout << "Mode must be one of:" << util::container_to_str(mode_set,"\n\t","\n\t","") << endl;
    }
    // check environment
    if(environment_set.find(environment_arg.getValue())==environment_set.end()) {
        ok = false;
        cout << "Environment must be one of:" << util::container_to_str(environment_set,"\n\t","\n\t","") << endl;
    }
    // check sample number
    if(sample_n_arg.getValue()<0) {
        ok = false;
        cout << "Number of samples must be non-negative." << endl;
    }
    // check graph type
    if(graph_type_set.find(graph_type_arg.getValue())==graph_type_set.end()) {
        ok = false;
        cout << "Graph type must be one of:" << util::container_to_str(graph_type_set,"\n\t","\n\t","") << endl;
    }
    // check backup type
    if(backup_type_set.find(backup_type_arg.getValue())==backup_type_set.end()) {
        ok = false;
        cout << "Backup type must be one of:" << util::container_to_str(backup_type_set,"\n\t","\n\t","") << endl;
    }
    // check rollout storage
    if(rollout_storage_set.find(rollout_storage_arg.getValue())==rollout_storage_set.end()) {
        ok = false;
        cout << "Rollout storage must be one of:" << util::container_to_str(rollout_storage_set,"\n\t","\n\t","") << endl;
    }
    // check backup method
    if(backup_method_set.find(backup_method_arg.getValue())==backup_method_set.end()) {
        ok = false;
        cout << "Backup method must be one of:" << util::container_to_str(backup_method_set,"\n\t","\n\t","") << endl;
    }
    // check value heuristic
    if(value_heuristic_set.find(value_heuristic_arg.getValue())==value_heuristic_set.end()) {
        ok = false;
        cout << "value heuristic must be one of:" << util::container_to_str(value_heuristic_set,"\n\t","\n\t","") << endl;
    }
    // check tree policy
    if(tree_policy_set.find(tree_policy_arg.getValue())==tree_policy_set.end()) {
        ok = false;
        cout << "Tree policy must be one of:" << util::container_to_str(tree_policy_set,"\n\t","\n\t","") << endl;
    }
    // check action policy
    if(tree_policy_set.find(action_policy_arg.getValue())==tree_policy_set.end()) {
        ok = false;
        cout << "Action policy must be one of:" << util::container_to_str(tree_policy_set,"\n\t","\n\t","") << endl;
    }
    // check backup policy
    if(tree_policy_set.find(backup_policy_arg.getValue())==tree_policy_set.end()) {
        ok = false;
        cout << "Backup policy must be one of:" << util::container_to_str(tree_policy_set,"\n\t","\n\t","") << endl;
    }
    // check graphics type
    if(graphics_type_set.find(graphics_type_arg.getValue())==graphics_type_set.end()) {
        ok = false;
        cout << "Graphics type must be one of:" << util::container_to_str(graphics_type_set,"\n\t","\n\t","") << endl;
    }
    if(discount_arg.getValue()<0 || discount_arg.getValue()>1) {
        ok = false;
        cout << "Discount must be in [0:1]" << endl;
    }
    if(T_action_arg.getValue()<0) {
        ok = false;
        cout << "Temperature must be positive in " << T_action_arg.getName() << endl;
    }
    if(T_tree_arg.getValue()<0) {
        ok = false;
        cout << "Temperature must be positive in " << T_tree_arg.getName() << endl;
    }
    if(T_backup_arg.getValue()<0) {
        ok = false;
        cout << "Temperature must be positive in " << T_backup_arg.getName() << endl;
    }
    if(exploration_arg.getValue()<0) {
        ok = false;
        cout << "Exploration must be positive" << endl;
    }
    return ok;
}

shared_ptr<AbstractEnvironment> get_environment() {
    shared_ptr<AbstractEnvironment> environment;
    if(environment_arg.getValue()=="SimpleEnvironment") {
        environment.reset(new SimpleEnvironment());
    } else if(environment_arg.getValue()=="GamblingHall") {
        environment.reset(new GamblingHall(5, 1));
    } else if(environment_arg.getValue()=="DLVSOR") {
        environment.reset(new DelayedLowVarianceSubOptimalReward(5,3,0.4));
    } else if(environment_arg.getValue()=="LVSOR") {
        environment.reset(new LowVarianceSubOptimalReward(0.4));
    } else if(environment_arg.getValue()=="MCVSDP") {
        environment.reset(new MC_versus_DP());
    } else if(environment_arg.getValue()=="Stochastic1D") {
        environment.reset(new Stochastic1D(1,10,0.6,0.5,true));
    } else if(environment_arg.getValue()=="NastyStochastic1D") {
        environment.reset(new NastyStochastic1D(3,3,1,0.5,true));
    } else if(environment_arg.getValue()=="FOL") {
        environment = InterfaceMarc::makeAbstractEnvironment(new FOL_World("boxes_new.kvg"));
    } else if(environment_arg.getValue()=="BottleneckEnvironment") {
        environment.reset(new BottleneckEnvironment(9,2));
    } else {
        cout << "Unknown environment" << endl;
        DEBUG_DEAD_LINE;
    }
    return environment;
}

shared_ptr<NodeFinder> get_node_finder() {
    if(graph_type_arg.getValue()=="PlainTree")
        return shared_ptr<NodeFinder>(new PlainTree());
    if(graph_type_arg.getValue()=="FullDAG")
        return shared_ptr<NodeFinder>(new FullDAG());
    if(graph_type_arg.getValue()=="FullGraph")
        return shared_ptr<NodeFinder>(new FullGraph());
    if(graph_type_arg.getValue()=="ObservationTree")
        return shared_ptr<NodeFinder>(new ObservationTree());
    DEBUG_DEAD_LINE;
    return shared_ptr<NodeFinder>(new PlainTree());
}

shared_ptr<TreePolicy> get_policy(int type) {
    shared_ptr<TreePolicy> tree_policy;
    QString type_str = "";
    double temperature;
    auto policy_str = tree_policy_arg.getValue();
    switch(type) {
    case 0:
        type_str = "tree";
        temperature = T_tree_arg.getValue();
        policy_str = tree_policy_arg.getValue();
        break;
    case 1:
        type_str = "action";
        temperature = T_action_arg.getValue();
        policy_str = action_policy_arg.getValue();
        break;
    case 2:
        type_str = "backup";
        temperature = T_backup_arg.getValue();
        policy_str = backup_policy_arg.getValue();
        break;
    default:
        temperature = 0;
        DEBUG_DEAD_LINE;
    }
    if(policy_str=="UCB1") {
        auto policy = new UCB1(exploration_arg.getValue());
        if(mode_arg.getValue()=="WATCH") {
            commander.add_command({"set exploration "+type_str,"set ex "+type_str[0]}, [policy](double ex)->Ret{
                    policy->set_exploration(ex);
                    return {true,QString("Set exploration to %1").arg(ex)};
                }, "Set exploration for UCB1 policy ("+type_str+"-policy)");
        }
        policy->soft_max_temperature = temperature;
        tree_policy.reset(policy);
    } else if(policy_str=="UCB_Variance") {
        auto policy = new UCB_Variance(exploration_arg.getValue(),exploration_arg.getValue());
        if(exploration_arg.getValue()==0.707) {
            delete policy;
            policy = new UCB_Variance();
        }
        if(mode_arg.getValue()=="WATCH") {
            commander.add_command({"set exploration "+type_str,"set ex "+type_str[0]}, [policy](double ex)->Ret{
                    policy->set_exploration(ex);
                    return {true,QString("Set exploration to %1").arg(ex)};
                }, "Set exploration for UCB_Variance policy ("+type_str+"-policy)");
        }
        policy->soft_max_temperature = temperature;
        tree_policy.reset(policy);
    } else if(policy_str=="Quantile") {
        auto environment = get_environment();
        auto policy = new Quantile(exploration_arg.getValue(),
                                   quantile_arg.getValue());
        if(environment->has_max_reward() && environment->has_min_reward() && discount_arg.getValue()<1) {
            double min_return = environment->min_reward()/(1-discount_arg.getValue());
            double max_return = environment->max_reward()/(1-discount_arg.getValue());
            double prior_counts = prior_counts_arg.getValue()<0?1:prior_counts_arg.getValue();
            delete policy;
            policy = new Quantile(exploration_arg.getValue(),
                                  quantile_arg.getValue(),
                                  min_return,
                                  max_return,
                                  prior_counts);
        }
        if(mode_arg.getValue()=="WATCH") {
            commander.add_command({"set exploration "+type_str,"set ex "+type_str[0]}, [policy](double ex)->Ret{
                    policy->set_exploration(ex);
                    return {true,QString("Set exploration to %1").arg(ex)};
                }, "Set exploration for Quantile policy ("+type_str+"-policy)");
        }
        policy->soft_max_temperature = temperature;
        tree_policy.reset(policy);
    } else if(policy_str=="Uniform") {
        tree_policy.reset(new Uniform());
    } else if(policy_str=="HardUpper") {
        auto policy = new HardUpper();
        policy->soft_max_temperature = temperature;
        tree_policy.reset(policy);
    } else if(policy_str=="Optimal") {
        auto policy = new Optimal();
        policy->soft_max_temperature = temperature;
        tree_policy.reset(policy);
    } else DEBUG_DEAD_LINE;
    return tree_policy;
}

shared_ptr<ValueHeuristic> get_value_heuristic() {
    shared_ptr<ValueHeuristic> value_heuristic;
    if(value_heuristic_arg.getValue()=="Zero") {
        value_heuristic.reset(new Zero());
    } else if(value_heuristic_arg.getValue()=="RolloutStatistics") {
        value_heuristic.reset(new RolloutStatistics(prior_counts_arg.getValue()));
    } else DEBUG_DEAD_LINE;
    return value_heuristic;
}

shared_ptr<BackupMethod> get_backup_method() {
    shared_ptr<BackupMethod> backup_method;
    if(backup_method_arg.getValue()=="Bellman") {
        backup_method.reset(new Bellman(get_policy(2),prior_counts_arg.getValue()));
    } else if(backup_method_arg.getValue()=="MonteCarlo") {
        backup_method.reset(new MonteCarlo(prior_counts_arg.getValue()));
    } else if(backup_method_arg.getValue()=="HybridMCDP") {
        backup_method.reset(new HybridMCDP(0.9,
                                           prior_counts_arg.getValue(),
                                           prior_counts_arg.getValue()));
    } else DEBUG_DEAD_LINE;
    return backup_method;
}

MonteCarloTreeSearch::BACKUP_TYPE get_backup_type() {
    if(backup_type_arg.getValue()=="TRACE") return MonteCarloTreeSearch::BACKUP_TYPE::TRACE;
    if(backup_type_arg.getValue()=="PROPAGATE") return MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE;
    DEBUG_DEAD_LINE;
    return MonteCarloTreeSearch::BACKUP_TYPE::TRACE;
}

MonteCarloTreeSearch::ROLLOUT_STORAGE get_rollout_storage() {
    if(rollout_storage_arg.getValue()=="NONE") return MonteCarloTreeSearch::ROLLOUT_STORAGE::NONE;
    if(rollout_storage_arg.getValue()=="CONDENSED") return MonteCarloTreeSearch::ROLLOUT_STORAGE::CONDENSED;
    if(rollout_storage_arg.getValue()=="FULL") return MonteCarloTreeSearch::ROLLOUT_STORAGE::FULL;
    DEBUG_DEAD_LINE;
    return MonteCarloTreeSearch::ROLLOUT_STORAGE::NONE;
}

tuple<shared_ptr<AbstractSearchTree>,
      shared_ptr<AbstractEnvironment>> setup() {
    shared_ptr<AbstractSearchTree>  search_tree;
    shared_ptr<TreePolicy>          tree_policy     = get_policy(0);
    shared_ptr<TreePolicy>          action_policy   = get_policy(1);
    shared_ptr<ValueHeuristic>      value_heuristic = get_value_heuristic();
    shared_ptr<BackupMethod>        backup_method   = get_backup_method();
    shared_ptr<AbstractEnvironment> environment     = get_environment();
    // set up search tree
    if(active_arg.getValue()) {
        search_tree.reset(new ActiveTreeSearch(environment,
                                               discount_arg.getValue(),
                                               get_node_finder()));
    } else if(random_arg.getValue()) {
        search_tree.reset(new RandomSearch(environment,
                                           discount_arg.getValue()));
    } else {
        auto search = new MonteCarloTreeSearch(environment,
                                               discount_arg.getValue(),
                                               get_node_finder(),
                                               tree_policy,
                                               value_heuristic,
                                               backup_method,
                                               get_backup_type(),
                                               rollout_length_arg.getValue(),
                                               action_policy,
                                               -1,
                                               get_rollout_storage());
        if(tree_policy_arg.getValue()=="Quantile") search->data_backups(true);
        search_tree.reset(search);
    }
    search_tree->init();
    if(mode_arg.getValue()=="WATCH") {
        // add commands for plotting tree
        commander.add_command("print tree",[search_tree]()->Ret{
                plot_graph(search_tree);
                return {true,QString("Printed search tree to '%1'").
                        arg(graph_plot_file_name).toLatin1()};
            }, "Print the search tree to PDF file 'tree.pdf'");
        commander.add_command("print tree",[search_tree](QString file_name)->Ret{
                plot_graph(search_tree, file_name.toLatin1());
                return {true,QString("Printed search tree to '%1'").arg(file_name)};
            }, "Print the search tree to PDF file with given name");

        // set graphics type
        if(graphics_type_arg.getValue()=="pdf") {
            graph_plot_file_name = "tree.pdf";
            graph_plot_command = "dot";
            graph_plot_parameters = "-Tpdf";
        } else if(graphics_type_arg.getValue()=="svg") {
            graph_plot_file_name = "tree.svg";
            graph_plot_command = "dot";
            graph_plot_parameters = "-Tsvg";
        } else if(graphics_type_arg.getValue()=="png") {
            graph_plot_file_name = "tree.png";
            graph_plot_command = "dot";
            graph_plot_parameters = "-Tpng";
        } else DEBUG_DEAD_LINE;
    }
    // return
    return make_tuple(search_tree,
                      environment);
}

QString header(int argn, char ** args) {
    QString str;
    // date and time
    str += "# Date/Time: ";
    str += QDateTime::currentDateTime().toString("yyyy-MM-dd_hh:mm:ss");
    // command line arguments
    str += "\n# Command: ";
    for(int arg=0; arg<argn; ++arg) {
        str += args[arg];
        if(arg<argn-1) {
            str += " ";
        }
    }
    return str;
}
