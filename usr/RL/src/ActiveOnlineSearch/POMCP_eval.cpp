#include <iostream>
#include <time.h>
#include <vector>
#include <set>
#include <tuple>
#include <unistd.h>
#include <memory>

#include <tclap/CmdLine.h>

#include <QDateTime>
#include <util/QtUtil.h>

#include "Environment/SimpleEnvironment.h"
#include "Environment/GamblingHall.h"
#include "Environment/BottleneckEnvironment.h"
#include "Environment/VarianceEnvironments.h"
#include "Environment/MC_versus_DP.h"
#include "Environment/Stochastic1D.h"
#include "../../../../share/src/FOL/fol_mcts_world.h"
#include "../../../../share/src/POMCP/mcts.h"

#include <omp.h>
//#define USE_OMP

#include <util/util.h>
#include <util/pretty_printer.h>

using std::cout;
using std::cin;
using std::endl;
using std::shared_ptr;
using std::make_shared;
using std::tuple;
using util::Range;

typedef AbstractEnvironment::action_handle_t      action_t;
typedef AbstractEnvironment::observation_handle_t observation_t;
typedef AbstractEnvironment::reward_t             reward_t;

static const std::set<std::string> mode_set = {"WATCH",
                                               "EVAL_ONLINE",
                                               "EVAL_ROOT_ACTION"};
static const std::set<std::string> environment_set = {"GamblingHall",
                                                      "FOL",
                                                      "SimpleEnvironment",
                                                      "BottleneckEnvironment",
                                                      "DLVSOR",
                                                      "LVSOR",
                                                      "MCVSDP",
                                                      "Stochastic1D"
};

// the command line arguments
static TCLAP::ValueArg<std::string> mode_arg(        "m", "mode",       \
                                                     "Mode to use "+util::container_to_str(mode_set,", ","(",")"), \
                                                     true, "WATCH", "string");
static TCLAP::ValueArg<std::string> environment_arg( "e", "environment", \
                                                     "Environment to use "+util::container_to_str(environment_set,", ","(",")"), \
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
static TCLAP::ValueArg<double> discount_arg(         "d", "discount", "(default: 1) Discount for the returns"
                                                     , false, 1, "double");
static TCLAP::ValueArg<int> random_seed_arg(         "", "random_seed", \
                                                     "(default: -1) Random seed to use. For negative values (default) current time is used as seed.", \
                                                     false, -1, "int" );
static TCLAP::SwitchArg no_header_arg(               "", "no_header",\
                                                     "(default: false) In EVAL mode (-m EVAL) don't print a head line containing the column names."\
                                                     , false);
static TCLAP::SwitchArg print_transitions_arg(       "", "print_transitions",\
                                                     "(default: false) In EVAL mode (-m EVAL) print the single transitions."\
                                                     , false);
static TCLAP::ValueArg<int> threads_arg(             "t", "threads", \
                                                     "(default: 0) Maximum number of threads to use by calling omp_set_num_threads(). A value of \
Zero (default) or below does not restict the number of threads so the \
OMP_NUM_THREADS environment variable will take effect (if set).", \
                                                     false, 0, "int" );

bool check_arguments();

shared_ptr<AbstractEnvironment> get_environment();

MCTS::PARAMS get_params(int samples = -1);

namespace std {
    ostream& operator<<(ostream & out, const MCTS::PARAMS & params) {
        out << "PARAMS(";
        out << "Verbose=" << params.Verbose << ";";
        out << "MaxDepth=" << params.MaxDepth << ";";
        if(mode_arg.getValue()=="EVAL_ONLINE" || mode_arg.getValue()=="EVAL_ROOT_ACTION") {
            out << "NumSimulations='number of roll-outs';";
        } else {
            out << "NumSimulations=" << params.NumSimulations << ";";
        }
        out << "NumStartStates=" << params.NumStartStates << ";";
        out << "UseTransforms=" << params.UseTransforms << ";";
        out << "NumTransforms=" << params.NumTransforms << ";";
        out << "MaxAttempts=" << params.MaxAttempts << ";";
        out << "ExpandCount=" << params.ExpandCount << ";";
        out << "ExplorationConstant=" << params.ExplorationConstant << ";";
        out << "UseRave=" << params.UseRave << ";";
        out << "RaveDiscount=" << params.RaveDiscount << ";";
        out << "RaveConstant=" << params.RaveConstant << ";";
        out << "DisableTree=" << params.DisableTree << ";";
        out << ")";
        return out;
    }
}

QString header(int argn, char ** args);

int main(int argn, char ** args) {

    // get command line arguments
    try {
	TCLAP::CmdLine cmd("Sample an evironment or perform online search", ' ', "");
        cmd.add(threads_arg);
        cmd.add(print_transitions_arg);
        cmd.add(no_header_arg);
        cmd.add(random_seed_arg);
        cmd.add(discount_arg);
        cmd.add(watch_progress_arg);
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
        // set up
        shared_ptr<AbstractEnvironment> environment = get_environment();
        MCTS::PARAMS SearchParams = get_params();
        MCTS pomcp(environment, SearchParams);
        pomcp.SetDiscount(discount_arg.getValue());
        action_t action;
        observation_t observation;
        reward_t reward;
        cout << "Watch progress..." << endl;
        cout << "Method: POMCP(Environment=" << *environment << ";parameters=" << SearchParams << ";)" << endl;
        // perform steps
        for(int step=0; step<step_n_arg.getValue() || step_n_arg.getValue()<0; ++step) {
            // planning
            environment->reset_state();
            if(environment->is_terminal_state()) break;
            pomcp.UCTSearch();
            int best_action_idx = pomcp.SelectAction();
            // make a transition
            environment->reset_state();
            assert(!environment->is_terminal_state());
            action = environment->get_actions()[best_action_idx];
            tuple<observation_t&,reward_t&>(observation,reward) = environment->transition(action);
            environment->make_current_state_default();
            if(watch_progress_arg.getValue()>=2) {
                cout << "Step # " << step+1 <<
                    ": (action --> observation, reward) = (" <<
                    *action << " --> " <<
                    *observation << ", " <<
                    reward << ")" << endl;
                if(environment->is_terminal_state()) {
                    cout << "Terminal state!" << endl;
                }
                getchar();
            }
        }
            } else if(mode_arg.getValue()=="EVAL_ONLINE") {
        // print header
        if(!no_header_arg.getValue()) {
            cout << "mean reward,number of roll-outs,run,method" << endl;
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

                // set up
                shared_ptr<AbstractEnvironment> environment = get_environment();
                auto SearchParams = get_params(sample);
                MCTS pomcp(environment, SearchParams);
                pomcp.SetDiscount(discount_arg.getValue());
                action_t action;
                observation_t observation;
                reward_t reward;
                double reward_sum = 0;
                // for each number of samples a number of steps (or until
                // terminal state) is performed
                int step = 1;
                while(!environment->is_terminal_state()) {
                    // planning
                    environment->reset_state();
                    int best_action_idx = pomcp.SelectAction();
                    // make a transition
                    environment->reset_state();
                    assert(!environment->is_terminal_state());
                    action = environment->get_actions()[best_action_idx];
                    tuple<observation_t&,reward_t&>(observation,reward) = environment->transition(action);
                    environment->make_current_state_default();
                    reward_sum += reward;
#ifdef USE_OMP
#pragma omp critical (MCTS)
#endif
                    if(print_transitions_arg.getValue()) {
                        cout << run << "	" <<
                            *action << "	" <<
                            *observation << "	" <<
                            reward << endl;
                    }
                    // break on terminal state
                    if(environment->is_terminal_state()) break;
                    // break if (maximum) number of steps was set and reached
                    if(step_n_arg.getValue()>=0 && step>=step_n_arg.getValue()) break;
                    // increment step number
                    ++step;
                }
#ifdef USE_OMP
#pragma omp critical (MCTS)
#endif
                {
                    cout << QString("%1,%2,%3,").
                        arg(reward_sum/step).
                        arg(sample).
                        arg(run);
                    cout << "POMCP(Environment=" << *environment << ";parameters=" <<
                        SearchParams << ";)" << endl;
                }
            }
        }
    } else if(mode_arg.getValue()=="EVAL_ROOT_ACTION") {
        // print header
        if(!no_header_arg.getValue()) {
            cout << "mean reward,number of roll-outs,run,method" << endl;
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

                // set up
                shared_ptr<AbstractEnvironment> environment = get_environment();
                auto SearchParams = get_params(sample);
                MCTS pomcp(environment, SearchParams);
                pomcp.SetDiscount(discount_arg.getValue());
                action_t action;
                // planning
                environment->reset_state();
                int best_action_idx = pomcp.SelectAction();
                // make a transition
                environment->reset_state();
                assert(!environment->is_terminal_state());
                action = environment->get_actions()[best_action_idx];
#ifdef USE_OMP
#pragma omp critical (MCTS)
#endif
                {
                    cout << *action << QString(",	%2,	%3,	").
                        arg(sample).
                        arg(run);
                    cout << "POMCP(Environment=" << *environment << ";parameters=" <<
                        SearchParams << ";)" << endl;
                }
            }
        }
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
    if(discount_arg.getValue()<0 || discount_arg.getValue()>1) {
        ok = false;
        cout << "Discount must be in [0:1]" << endl;
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
        environment.reset(new Stochastic1D(10,0.6,0.5,true));
    } else if(environment_arg.getValue()=="FOL") {
        environment = InterfaceMarc::makeAbstractEnvironment(new FOL_World("boxes_new.kvg"));
    } else if(environment_arg.getValue()=="BottleneckEnvironment") {
        environment.reset(new BottleneckEnvironment(9,2));
    } else {
        cout << "Unknown environment" << endl;
    }
    return environment;
}

MCTS::PARAMS get_params(int samples) {
    MCTS::PARAMS params;
    if(samples!=-1) {
        params.NumSimulations = samples;
    } else {
        params.NumSimulations = sample_n_arg.getValue();
    }
    params.NumStartStates = 1;
    return params;
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
