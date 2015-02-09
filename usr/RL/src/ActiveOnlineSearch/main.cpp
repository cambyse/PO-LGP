#include <iostream>
#include <time.h>
#include <vector>
#include <set>
#include <tuple>
#include <unistd.h>

#include <tclap/CmdLine.h>

#include <util/util.h>
#include <util/QtUtil.h>

#include "SearchTree.h"
#include "TightRope.h"
#include "DynamicTightRope.h"

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::cout;
using std::endl;
using util::Range;

// the command line arguments
static TCLAP::ValueArg<std::string> mode_arg(        "m", "mode",\
                                                     "Mode to use. \n'SAMPLE' takes <n> (--sample_n <n>) random state-action samples \
from the environment <e> (--environment <e>) and prints the \
state-action-state-reward tuples to std::cout.\n'UCT' runs tree search with \
<n> rollouts per step and performs <s> (--step_n <s>) steps. The progress \
during tree building can be watched at a different level of detail \
(--progress).\n'UCT_EVAL' performs <r> (--run_n <r>) runs with a series of \
trials for different numbers of rollouts <n>, <n>+<i>, ... , <m> \
(--sample_incr <i> --sample_max <m>) and <s> steps per trial and prints the \
mean reward per trial to std::cout.",\
                                                     false, "SAMPLE", "string");
static TCLAP::ValueArg<std::string> environment_arg( "e", "environment",\
                                                     "Environment to use.",\
                                                     false, "DynamicTightRope", "string");
static TCLAP::ValueArg<int> sample_n_arg(            "n", "sample_n",\
                                                     "Number of samples/rollouts.",\
                                                     false, 1000, "int" );
static TCLAP::ValueArg<int> sample_incr_arg(         "i", "sample_incr",\
                                                     "Increment of the number rollouts. ",\
                                                     false, 1, "int" );
static TCLAP::ValueArg<int> sample_max_arg(          "", "sample_max",\
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
static const std::set<std::string> mode_set = {"SAMPLE",
                                               "UCT",
                                               "UCT_EVAL"};
static const std::set<std::string> environment_set = {"TightRope",
                                                      "DynamicTightRope"};

bool check_arguments();

int main(int argn, char ** args) {

    // get command line arguments
    try {
	TCLAP::CmdLine cmd("Sample an evironment or perform online search", ' ', "");
        cmd.add(no_graphics_arg);
        cmd.add(watch_progress_arg);
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
    srand(time(nullptr));
    srand48(time(nullptr));

    // setup environment
    std::shared_ptr<Environment> environment;
    if(environment_arg.getValue()=="TightRope") {
        environment.reset(new TightRope(15));
    } else if(environment_arg.getValue()=="DynamicTightRope") {
        environment.reset(new DynamicTightRope(15));
    } else {
        cout << "Unknown environment" << endl;
        DEBUG_DEAD_LINE;
    }

    // different modes
    if(mode_arg.getValue()=="SAMPLE") {
        repeat(sample_n_arg.getValue()) {
            Environment::state_t state = util::random_select(environment->states);
            Environment::action_t action = util::random_select(environment->actions);
            auto state_reward = environment->sample(state, action);
            cout << state << " " << action << " " << std::get<0>(state_reward) << " " << std::get<1>(state_reward) << endl;
        }
    } else if(mode_arg.getValue()=="UCT") {
        cout << "Running UCT..." << endl;
        SearchTree tree(0, environment, 0.5);
        for(int step : Range(0,step_n_arg.getValue())) {
            for(int sample : Range(sample_n_arg.getValue())) {
                tree.perform_rollout();
                if(watch_progress_arg.getValue()>=3 && !no_graphics_arg.getValue()) {
                    tree.toPdf("tree.pdf");
                    cout << "Sample # " << sample+1 << endl;
                    getchar();
                }
            }
            if(step<step_n_arg.getValue()) {
                auto action = tree.recommend_action();
                auto state_reward = environment->sample(tree.node_info_map[tree.root_node].state,action);
                auto state = std::get<0>(state_reward);
                tree.prune(action,state);
                if(watch_progress_arg.getValue()>=2 && !no_graphics_arg.getValue()) {
                    tree.toPdf("tree.pdf");
                    cout << "Step # " << step+1 << ": (action --> state) = (" << action << " --> " << state << ")" << endl;
                    getchar();
                }
            }
        }
        if(watch_progress_arg.getValue()>=1 && !no_graphics_arg.getValue()) {
            tree.toPdf("tree.pdf");
        }
    } else if(mode_arg.getValue()=="UCT_EVAL") {
        SearchTree tree(0, environment, 0.5);
        // print header
        cout << "mean reward,number of samples,run" << endl;
        // several runs
        for(int run : Range(run_n_arg.getValue())) {
            DEBUG_OUT(1, "Run # " << run);
            // with one trial for a different number of samples
            for(int sample_n : Range(sample_n_arg.getValue(), sample_max_arg.getValue(), sample_incr_arg.getValue())) {
                DEBUG_OUT(1, "Samples: " << sample_n);
                double mean_reward = 0;
                // for each number of samples a number of steps (or until
                // terminal state) is performed
                int step = 1;
                tree.init(0); // reinitialize tree to state 0
                while(true) {
                    // build tree
                    repeat(sample_n) {
                        tree.perform_rollout();
                    }
                    // perform step
                    auto action = tree.recommend_action();
                    auto state_reward = environment->sample(tree.node_info_map[tree.root_node].state,action);
                    auto state = std::get<0>(state_reward);
                    mean_reward += std::get<1>(state_reward);
                    // break on terminal state
                    if(environment->has_terminal_state() && environment->is_terminal_state(state)) break;
                    // break if (maximum) number of steps was set and reached
                    if(step_n_arg.getValue()>0 && step>=step_n_arg.getValue()) break;
                    // otherwise prune tree and increment step number
                    tree.prune(action,state);
                    ++step;
                }
                cout << QString("%1,%2,%3").arg(mean_reward/step).arg(sample_n).arg(run) << endl;
            }
        }
    } else {
        cout << "Unknown mode" << endl;
        DEBUG_DEAD_LINE;
    }
}

bool check_arguments() {
    bool ok = true;
    // check mode
    if(mode_set.find(mode_arg.getValue())==mode_set.end()) {
        ok = false;
        cout << "Mode must be one of:" << endl;
        for(auto mode : mode_set) {
            cout << "\t" << mode << endl;
        }
    }
    // check environment
    if(environment_set.find(environment_arg.getValue())==environment_set.end()) {
        ok = false;
        cout << "Environment must be one of:" << endl;
        for(auto environment : environment_set) {
            cout << "\t" << environment << endl;
        }
    }
    // check sample number
    if(sample_n_arg.getValue()<0) {
        ok = false;
        cout << "Number of samples must be non-negative." << endl;
    }
    return ok;
}
