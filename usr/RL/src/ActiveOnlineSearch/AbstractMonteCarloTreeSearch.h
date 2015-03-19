#ifndef ABSTRACTMONTECARLOTREESEARCH_H_
#define ABSTRACTMONTECARLOTREESEARCH_H_

#include "SearchTree.h"

class AbstractMonteCarloTreeSearch: public SearchTree {

    //----typedefs/classes----//

public:

    /**
     * Node-specific data for MCTS.*/
    struct MCTSNodeInfo {
        /**
         * Nr of times this node (state or action) was sampled. */
        int counts = 0;
        /**
         * Value of this action/state. */
        double value = 0;
    };
    typedef graph_t::NodeMap<MCTSNodeInfo> mcts_node_info_map_t;

    /**
     * Arc-specific data for MCTS.*/
    struct MCTSArcInfo {
        /**
         * Only valid for action --> state arcs: number of times this
         * transition occurred. */
        int counts = 0;
        /**
         * Only valid for action --> state arcs: sum of all rewards for this
         * transition. */
        double mean_reward = 0;
        /**
         * Only valid for action --> state arcs: probability for this
         * transition to occurred */
        double probability = 0;
    };
    typedef graph_t::ArcMap<MCTSArcInfo>   mcts_arc_info_map_t;

    //----members----//
protected:
    /**
     * Map holding node-specific information of type MCTSNodeInfo . */
    mcts_node_info_map_t mcts_node_info_map;
    /**
     * Map holding arc-specific information of type MCTSArcInfo. */
    mcts_arc_info_map_t mcts_arc_info_map;

    //----methods----//
public:
    AbstractMonteCarloTreeSearch(const state_t & root_state,
                                 std::shared_ptr<Environment> environment,
                                 double discount):
    SearchTree(root_state, environment, discount),
        mcts_node_info_map(graph),
        mcts_arc_info_map(graph){}
    virtual ~AbstractMonteCarloTreeSearch() = default;
};

#endif /* ABSTRACTMONTECARLOTREESEARCH_H_ */
