#include "ValueHeuristic.h"

namespace value_heuristic {

    ValueHeuristic::ValueHeuristic(std::shared_ptr<const Environment> environment,
                                   mcts_node_info_map_t & mcts_node_info_map):
        environment(environment), mcts_node_info_map(mcts_node_info_map)
    {}

    Zero::Zero(std::shared_ptr<const Environment> environment,
               mcts_node_info_map_t & mcts_node_info_map):
        ValueHeuristic(environment, mcts_node_info_map)
    {}

    void Zero::get_value(const node_t & node) const {
        mcts_node_info_map[node].value = 0;
    }

} // end namespace value_heuristic

