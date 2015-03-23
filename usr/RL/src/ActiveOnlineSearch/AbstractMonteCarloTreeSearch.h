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
        /**
         * Trace to predecessor node on the last rollout that visited this
         * node. The tuple consists of (state_node_from, arc_to_action,
         * action_node, arc_to_state) which led to \e this node. */
        std::tuple<node_t,arc_t,node_t,arc_t> backtrace;
        MCTSNodeInfo();
    };
    typedef graph_t::NodeMap<MCTSNodeInfo> mcts_node_info_map_t;

    /**
     * Arc-specific data for MCTS.*/
    struct MCTSArcInfo {
        /**
         * Number of times this arc was taken on a rollout. For action-->state
         * arcs this corresponds to the (unnormalized) transition
         * probability. For state-->action arcs this correpsonds to the
         * (unnormalized) policy. */
        int counts = 0;
        /**
         * Sum of rewards of all transitions taking this arc. For action-->state
         * arcs this corresponds to the (unnormalized) expected reward as a
         * function of action and target-state. For state-->action arcs this
         * corresponds to the (unnormalized) expected reward as a function of
         * source-state and action. */
        double reward_sum = 0;
    };
    typedef graph_t::ArcMap<MCTSArcInfo>   mcts_arc_info_map_t;

    //----members----//
protected:
    /**
     * Const reference to graph of SearchTree basis class. */
    const graph_t & graph;
    /**
     * Map holding node-specific information of type MCTSNodeInfo . */
    mcts_node_info_map_t mcts_node_info_map;
    /**
     * Map holding arc-specific information of type MCTSArcInfo. */
    mcts_arc_info_map_t mcts_arc_info_map;

    //----methods----//
public:
    AbstractMonteCarloTreeSearch(const state_t & root_state,
                                 Environment & environment,
                                 double discount,
                                 GRAPH_TYPE graph_type);
    virtual ~AbstractMonteCarloTreeSearch() = default;
    virtual void prune(const action_t & a, const state_t & s) override;
    void toPdf(const char* file_name) const override;
};

#endif /* ABSTRACTMONTECARLOTREESEARCH_H_ */
