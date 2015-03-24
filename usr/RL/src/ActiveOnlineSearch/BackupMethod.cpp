#include "BackupMethod.h"

#include <float.h>
#include <algorithm>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using lemon::INVALID;

namespace backup_method {

    void Bellman::operator()(const node_t & state_node,
                             const node_t & action_node,
                             double discount,
                             std::shared_ptr<const Environment> environment,
                             const graph_t & graph,
                             mcts_node_info_map_t & mcts_node_info_map,
                             const mcts_arc_info_map_t & mcts_arc_info_map) const {
        // compute action value
        {
            reward_t value = 0;
            reward_t value_err = 0;
            for(out_arc_it_t arc(graph, action_node); arc!=INVALID; ++arc) {
                value += mcts_arc_info_map[arc].get_counts()/mcts_node_info_map[action_node].get_transition_counts() * // transition probability
                    (mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_counts() +                     // mean reward
                     discount*mcts_node_info_map[graph.target(arc)].get_value());                                      // discounted value of next state
            }
            mcts_node_info_map[action_node].set_value(value);
        }

        // compute state value
        {
            reward_t max_value = -DBL_MAX;
            for(out_arc_it_t arc(graph, state_node); arc!=INVALID; ++arc) {
                max_value = std::max(max_value, mcts_node_info_map[graph.target(arc)].get_value());
            }
            mcts_node_info_map[state_node].set_value(max_value);
        }
    }

    void MonteCarlo::operator()(const node_t & state_node,
                                const node_t & action_node,
                                double discount,
                                std::shared_ptr<const Environment> environment,
                                const graph_t & graph,
                                mcts_node_info_map_t & mcts_node_info_map,
                                const mcts_arc_info_map_t & mcts_arc_info_map) const {
        auto& action_info = mcts_node_info_map[action_node];
        action_info.set_value(action_info.get_return_sum()/action_info.get_rollout_counts());
        auto& state_info = mcts_node_info_map[state_node];
        state_info.set_value(state_info.get_return_sum()/state_info.get_rollout_counts());
        DEBUG_OUT(1,QString("    backup state-node(%1):	value=%2").
                  arg(graph.id(state_node)).
                  arg(mcts_node_info_map[state_node].get_value()));
        DEBUG_OUT(1,QString("    backup action-node(%1):	value=%2").
                  arg(graph.id(action_node)).
                  arg(mcts_node_info_map[action_node].get_value()));
    }

} // namespace backup_method {
