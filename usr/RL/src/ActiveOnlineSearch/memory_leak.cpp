#include <iostream>
#include <time.h>
#include <tuple>
#include <memory>

#include "Environment/MC_versus_DP.h"
#include "../../../../share/src/POMCP/mcts.h"

int main(int argn, char ** args) {

    auto seed = time(nullptr);
    srand(seed);
    srand48(seed);

    int run_n = 1000;
    int sample_min = 200;
    int sample_max = 1000;
    int sample_incr = 200;

    // header
    std::cout << "mean reward,	number of roll-outs,	run,	method" << std::endl;

    // several runs
    for(int run=0; run<run_n; ++run) {

        // different number of samples
        for(int sample=sample_min; sample<=sample_max; sample+=sample_incr) {

            MCTS::PARAMS SearchParams;
            SearchParams.NumStartStates = 1;
            SearchParams.NumSimulations = sample;
            std::shared_ptr<AbstractEnvironment> environment(new MC_versus_DP());
            MCTS pomcp(environment, SearchParams);
            pomcp.SetDiscount(1);

            // perform planned steps until reaching a terminal state
            double reward_sum = 0;
            int step = 0;
            for(; !environment->is_terminal_state(); ++step) {

                // planning
                environment->reset_state();
                pomcp.UCTSearch();
                int best_action_idx = pomcp.SelectAction();

                // make a transition
                environment->reset_state();
                assert(!environment->is_terminal_state());
                auto action = environment->get_actions()[best_action_idx];
                auto observation_and_reward = environment->transition(action);
                reward_sum += std::get<1>(observation_and_reward);
                environment->make_current_state_default();
            }

            // print output
            std::cout << reward_sum/step << ",	"
                      << sample << ",	"
                      << run << ",	POMCP" << std::endl;
        }
    }
}
