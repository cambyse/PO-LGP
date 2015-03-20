#ifndef TREEPOLICY_H_
#define TREEPOLICY_H_

#include "AbstractMonteCarloTreeSearch.h"

class Environment;

namespace tree_policy {

    typedef AbstractMonteCarloTreeSearch::graph_t              graph_t;
    typedef AbstractMonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::mcts_arc_info_map_t  mcts_arc_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_t               node_t;
    typedef AbstractMonteCarloTreeSearch::action_t             action_t;

    /**
     * Abstract basis class for tree policies. The job of the tree policy is to
     * traverse the existing search tree until reaching a leaf node. The most
     * common tree policy is UCB1 but a Uniform policy may also be used. */
    class TreePolicy {
    public:
        virtual action_t operator()(const node_t & state_node,
                                    const Environment & environment,
                                    const graph_t & graph,
                                    const node_info_map_t & node_info_map,
                                    const mcts_node_info_map_t & mcts_node_info_map,
                                    const mcts_arc_info_map_t & mcts_arc_info_map) const = 0;
    };

    /**
     * Sample actions uniformly from available action nodes. */
    class Uniform: public TreePolicy {
    public:
        virtual action_t operator()(const node_t & state_node,
                                    const Environment & environment,
                                    const graph_t & graph,
                                    const node_info_map_t & node_info_map,
                                    const mcts_node_info_map_t & mcts_node_info_map,
                                    const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    };

    /**
     * Sample actions according to UCB1 policy. */
    class UCB1: public TreePolicy {
    public:
        /**
         * Constructor. @param Cp This is the scaling parameter for
         * exploration. The default value is Cp=1/√2, which was shown by Kocsis
         * and Szepesvári to satisfy the Hoeffding ineqality. */
        UCB1(double Cp = 0.70710678118654746);
        virtual action_t operator()(const node_t & state_node,
                                    const Environment & environment,
                                    const graph_t & graph,
                                    const node_info_map_t & node_info_map,
                                    const mcts_node_info_map_t & mcts_node_info_map,
                                    const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    protected:
        double Cp;
    };

} // end namespace tree_policy

#endif /* TREEPOLICY_H_ */
