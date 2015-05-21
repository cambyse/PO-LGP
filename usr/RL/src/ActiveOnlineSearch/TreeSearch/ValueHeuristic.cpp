#include "ValueHeuristic.h"

#include "../Environment/Environment.h"

#include <limits>

#include <util/util.h>
#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <util/return_tuple_macros.h>

namespace value_heuristic {

    void Zero::operator()(const node_t & state_node,
                          const state_handle_t &,
                          double,
                          std::shared_ptr<AbstractEnvironment>,
                          mcts_node_info_map_t & mcts_node_info_map) const {
        mcts_node_info_map[state_node].add_rollout_return(0);
        mcts_node_info_map[state_node].set_value(0,0);
    }

    Rollout::Rollout(int rollout_length): rollout_length(rollout_length) {}

    void Rollout::operator()(const node_t & state_node,
                             const state_handle_t & start_state,
                             double discount,
                             std::shared_ptr<AbstractEnvironment> environment,
                             mcts_node_info_map_t & mcts_node_info_map) const {

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
        DEBUG_OUT(1,"Rollout from state '" << Environment::name(*environment,start_state) << "'");
        environment->set_state(start_state);
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
            DEBUG_OUT(2,"    action '" << Environment::name(*environment,action) <<
                      "'	--> state '" << Environment::name(*environment,environment->get_state_handle()) << "':	r=" << reward <<
                      "	R=" << discounted_return << "	d=" << discount_factor);
        }
        mcts_node_info_map[state_node].add_rollout_return(discounted_return);
        mcts_node_info_map[state_node].set_value(discounted_return,
                                                 environment->is_terminal_state(start_state)?
                                                 0:
                                                 std::numeric_limits<double>::infinity());
    }

} // end namespace value_heuristic
