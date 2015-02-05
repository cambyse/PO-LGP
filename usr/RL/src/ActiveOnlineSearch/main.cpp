#include <iostream>
#include <time.h>
#include <set>
#include <unistd.h>

#include <tclap/CmdLine.h>

#include <util/util.h>

#include "SearchTree.h"
#include "TightRope.h"

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::cout;
using std::endl;

// the command line arguments
static TCLAP::ValueArg<std::string> mode_arg(        "m", "mode",\
                                                     "mode to use",\
                                                     false, "Sample", "string");
static TCLAP::ValueArg<std::string> environment_arg( "e", "environment",\
                                                     "environment to use",\
                                                     false, "TightRope", "string");
static TCLAP::ValueArg<int> sample_n_arg(            "n", "sample_n",\
                                                     "number of samples/rollouts",\
                                                     false, 1000,        "int"   );
static TCLAP::ValueArg<int> step_n_arg(              "s", "step_n",\
                                                     "number of steps to perform",\
                                                     false, 0,           "int"   );
static TCLAP::ValueArg<int> watch_progress_arg(      "p", "progress",\
                                                     "level of detail for watching progress (0,...,3)",\
                                                     false, 1, "int");
static TCLAP::SwitchArg no_graphics_arg(             "g", "no_graphics",\
                                                     "don't generate graphics"\
                                                     , false);
static const std::set<std::string> mode_set = {"Sample",
                                               "UCT"};
static const std::set<std::string> environment_set = {"TightRope"};

bool check_arguments();

int main(int argn, char ** args) {

    // get command line arguments
    try {
	TCLAP::CmdLine cmd("Sample an evironment or perform online search", ' ', "");
        cmd.add(no_graphics_arg);
        cmd.add(watch_progress_arg);
        cmd.add(step_n_arg);
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
    } else {
        cout << "Unknown environment" << endl;
        DEBUG_DEAD_LINE;
    }

    // different modes
    if(mode_arg.getValue()=="Sample") {
        repeat(sample_n_arg.getValue()) {
            Environment::state_t state = util::random_select(environment->states);
            Environment::action_t action = util::random_select(environment->actions);
            auto state_reward = environment->sample(state, action);
            cout << state << " " << action << " " << std::get<0>(state_reward) << " " << std::get<1>(state_reward) << endl;
        }
    } else if(mode_arg.getValue()=="UCT") {
        cout << "Running UCT..." << endl;
        SearchTree tree(0, environment, 0.5);
        for(int step : util::Range(0,step_n_arg.getValue())) {
            for(int sample : util::Range(sample_n_arg.getValue())) {
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
