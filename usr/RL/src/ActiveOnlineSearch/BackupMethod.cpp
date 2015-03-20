#include "BackupMethod.h"

#include <float.h>
#include <algorithm>

using lemon::INVALID;

namespace backup_method {

    void Bellman::operator()(const node_t & state_node,
                             const node_t & action_node,
                             double discount,
                             const Environment & environment,
                             const graph_t & graph,
                             mcts_node_info_map_t & mcts_node_info_map,
                             const mcts_arc_info_map_t & mcts_arc_info_map) const {
        // compute action value
        {
            reward_t value = 0;
            for(out_arc_it_t arc(graph, action_node); arc!=INVALID; ++arc) {
                value += (mcts_arc_info_map[arc].reward_sum +
                          discount*mcts_node_info_map[graph.target(arc)].value*mcts_arc_info_map[arc].counts
                    )/mcts_node_info_map[action_node].counts;
            }
            mcts_node_info_map[action_node].value = value;
        }

        // compute state value
        {
            reward_t max_value = -DBL_MAX;
            for(out_arc_it_t arc(graph, state_node); arc!=INVALID; ++arc) {
                max_value = std::max(max_value, mcts_node_info_map[graph.target(arc)].value);
            }
            mcts_node_info_map[state_node].value = max_value;
        }
    }

} // namespace backup_method {
