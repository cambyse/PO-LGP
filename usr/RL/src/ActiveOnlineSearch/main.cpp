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
#include "TightRope.h"
#include "DynamicTightRope.h"

#include <util/return_tuple.h>
#include <util/pretty_printer.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::cout;
using std::endl;
using util::Range;

static const std::set<std::string> mode_set = {"SAMPLE",
                                               "UCT",
                                               "UCT_EVAL"};
static const std::set<std::string> environment_set = {"TightRope",
                                                      "DynamicTightRope"};
static const std::set<std::string> accumulate_set = {"min",
                                                     "mean",
                                                     "max"};

// the command line arguments
static TCLAP::ValueArg<std::string> mode_arg(        "m", "mode",\
                                                     "Mode to use "+util::container_to_str(mode_set,", ","(",")")+".\n'SAMPLE' takes <n> (--sample_n <n>) samples for different state-action \
pairs from the environment <e> (--environment <e>) and prints the data to \
std::cout.\n'UCT' runs tree search with <n> rollouts per step and performs <s> \
(--step_n <s>) steps. The progress during tree building can be watched at a \
different level of detail (--progress).\n'UCT_EVAL' performs <r> (--run_n <r>) \
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
static TCLAP::ValueArg<std::string> accumulate_arg(  "a", "accumulate", \
                                                     "How to accumulate values "+util::container_to_str(accumulate_set,", ","(",")")+"."\
                                                     , false, "mean", "string");

bool check_arguments();

int main(int argn, char ** args) {

    // get command line arguments
    try {
	TCLAP::CmdLine cmd("Sample an evironment or perform online search", ' ', "");
        cmd.add(accumulate_arg);
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
        environment.reset(new DynamicTightRope(30));
    } else {
        cout << "Unknown environment" << endl;
        DEBUG_DEAD_LINE;
    }
    typedef Environment::state_t state_t;
    typedef Environment::action_t action_t;
    typedef Environment::reward_t reward_t;

    // different modes
    if(mode_arg.getValue()=="SAMPLE") {
        if(environment_arg.getValue()=="TightRope") {
            cout << "run,state,action,reward" << endl;
            for(int run : Range(sample_n_arg.getValue())) {
                for(auto state_from : environment->states) {
                    for(auto action : environment->actions) {
                        RETURN_TUPLE(state_t,state_to,reward_t,reward) = environment->sample(state_from, action);
                        cout << QString("%1,%2,%3,%4").arg(run).arg(state_from).arg(action).arg(reward) << endl;
                    }
                }
            }
        } else if(environment_arg.getValue()=="DynamicTightRope") {
            cout << "run,position,velocity,"+accumulate_arg.getValue()+"_reward" << endl;
            for(int run : Range(sample_n_arg.getValue())) {
                for(auto state_from : environment->states) {
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
                    for(auto action : environment->actions) {
                        RETURN_TUPLE(state_t,state_to,reward_t,reward) = environment->sample(state_from, action);
                        if(accumulate_arg.getValue()=="min") {
                            accum_reward = std::min(reward,accum_reward);
                        } else if(accumulate_arg.getValue()=="max") {
                            accum_reward = std::max(reward,accum_reward);
                        } else if(accumulate_arg.getValue()=="mean") {
                            accum_reward += reward/environment->actions.size();
                        } else {
                            DEBUG_DEAD_LINE;
                        }
                    }
                    cout << QString("%1,%2,%3,%4").arg(run).arg(position).arg(velocity).arg(accum_reward) << endl;
                }
            }
        }
    } else if(mode_arg.getValue()=="UCT") {
        cout << "Running UCT..." << endl;
        SearchTree tree(0, environment, 0.5);
        for(int step : Range(0,step_n_arg.getValue())) {
            for(int sample : Range(sample_n_arg.getValue())) {
                tree.perform_rollout();
                if(watch_progress_arg.getValue()>=3) {
                    if(!no_graphics_arg.getValue()) {
                        tree.toPdf("tree.pdf");
                    }
                    cout << "Sample # " << sample+1 << endl;
                    getchar();
                }
            }
            if(step<step_n_arg.getValue()) {
                auto action = tree.recommend_action();
                auto state_reward = environment->sample(tree.node_info_map[tree.root_node].state,action);
                auto state = std::get<0>(state_reward);
                tree.prune(action,state);
                if(watch_progress_arg.getValue()>=2) {
                    if(!no_graphics_arg.getValue()) {
                        tree.toPdf("tree.pdf");
                    }
                    cout << "Step # " << step+1 << ": (action --> state) = (" << environment->action_name(action) << " --> " << environment->state_name(state) << ")" << endl;
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
    return ok;
}
