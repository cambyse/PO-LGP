#include "ValueHeuristic.h"

namespace value_heuristic {

    void Zero::operator()(const node_t & state_node,
                          const Environment &,
                          mcts_node_info_map_t & mcts_node_info_map) const {
        mcts_node_info_map[state_node].value = 0;
    }

} // end namespace value_heuristic

