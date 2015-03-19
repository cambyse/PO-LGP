#ifndef TREEPOLICY_H_
#define TREEPOLICY_H_

#include "AbstractMonteCarloTreeSearch.h"

class Environment;

namespace tree_policy {

    typedef AbstractMonteCarloTreeSearch::graph_t              graph_t;
    typedef AbstractMonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_t               node_t;
    typedef AbstractMonteCarloTreeSearch::action_t             action_t;

    /**
     * Abstract basis class for tree policies. The job of the tree policy is to
     * traverse the existing search tree until reaching a leaf node. The most
     * common tree policy is UCB1 but a Uniform policy may also be used. */
    class TreePolicy {
        //----typedefs/classes----//

        //----members----//
    protected:
        std::shared_ptr<const Environment> environment;
        const graph_t & graph;
        const node_info_map_t & node_info_map;
        const mcts_node_info_map_t & mcts_node_info_map;
        //----methods----//
    public:
        TreePolicy(std::shared_ptr<const Environment> environment,
                   const graph_t & graph,
                   const node_info_map_t & node_info_map,
                   const mcts_node_info_map_t & mcts_node_info_map);
        virtual action_t next(const node_t &) = 0;
    };

    /**
     * Sample actions uniformly from available action nodes. */
    class Uniform: public TreePolicy {
        //----methods----//
    public:
        Uniform(std::shared_ptr<const Environment> environment,
                const graph_t & graph,
                const node_info_map_t & node_info_map,
                const mcts_node_info_map_t & mcts_node_info_map);
        action_t next(const node_t &) override;
    };

    /**
     * Sample actions according to UCB1 policy. */
    class UCB1: public TreePolicy {
        //----methods----//
    public:
        UCB1(std::shared_ptr<const Environment> environment,
             const graph_t & graph,
             const node_info_map_t & node_info_map,
             const mcts_node_info_map_t & mcts_node_info_map);
        action_t next(const node_t &) override;
    };

} // end namespace tree_policy

#endif /* TREEPOLICY_H_ */
