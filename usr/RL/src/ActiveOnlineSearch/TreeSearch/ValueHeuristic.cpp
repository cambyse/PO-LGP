#include "ValueHeuristic.h"

#include <limits>

#include <util/util.h>
#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <util/return_tuple_macros.h>

namespace value_heuristic {

    void ValueHeuristic::init(double disc,
                              std::shared_ptr<AbstractEnvironment> env) {
        discount = disc;
        environment = env;
    }

    void Zero::add_value_estimate(const node_t & state_node,
                                  mcts_node_info_map_t & mcts_node_info_map) const {
        mcts_node_info_map[state_node].add_rollout_return(0);
        // note: instead of adding zero we can as well leave the value unchanged
    }

    Rollout::Rollout(int rollout_length): rollout_length(rollout_length) {}

    void Rollout::add_value_estimate(const node_t & state_node,
                                     mcts_node_info_map_t & mcts_node_info_map) const {

        bool start_state_is_terminal = environment->is_terminal_state();

        // Do rollout of specified length (rollout_length).
        int length = rollout_length;

        // If rollout_length is not specified (smaller than zero) and
        // environment does not have a terminal state do one-step
        // rollout. Otherwise do infinite rollout (util terminal state).
        if(length<0 && !environment->has_terminal_state()) {
            length = 1;
        }
        observation_handle_t observation;
        action_handle_t action;
        reward_t reward;
        reward_t discounted_return = 0;
        double discount_factor = discount;
        int k=0;
        while((length<0 || k<length) && !environment->is_terminal_state()) {
            // select action uniformly from available actions
            action = util::random_select(environment->get_actions());
            // sample transition
            t(observation,reward) = environment->transition(action);
            // update return
            discounted_return += discount_factor*reward;
            // update counter and discount factor
            ++k;
            discount_factor*=discount;
        }
        mcts_node_info_map[state_node].add_rollout_return(discounted_return);

        auto rollout_return_sum = mcts_node_info_map[state_node].get_rollout_return_sum();
        auto squared_rollout_return_sum = mcts_node_info_map[state_node].get_squared_rollout_return_sum();
        auto rollout_counts = mcts_node_info_map[state_node].get_rollout_counts();
        DEBUG_EXPECT(0,rollout_counts>=1);
        if(rollout_counts==1) {
            mcts_node_info_map[state_node].set_value(rollout_return_sum/rollout_counts,
                                                     start_state_is_terminal?0:std::numeric_limits<double>::infinity());
        } else {
            mcts_node_info_map[state_node].set_value(rollout_return_sum/rollout_counts,
                                                     (rollout_counts/(rollout_counts-1))*(squared_rollout_return_sum/rollout_counts-pow(rollout_return_sum/rollout_counts,2)));
        }
#define FORCE_DEBUG_LEVEL 1
        if(discounted_return!=0)
            DEBUG_OUT(1,"rollout return: " << discounted_return);
    }

} // end namespace value_heuristic
