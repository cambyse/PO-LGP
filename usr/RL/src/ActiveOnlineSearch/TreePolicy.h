#ifndef TREEPOLICY_H_
#define TREEPOLICY_H_

#include "AbstractMonteCarloTreeSearch.h"

class Environment;

namespace tree_policy {

    typedef AbstractMonteCarloTreeSearch::graph_t graph_t;
    typedef AbstractMonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef SearchTree::node_info_map_t node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_t  node_t;

    /**
     * Abstract basis class for tree policies. The job of the tree policy is to
     * traverse the existing search tree until reaching a leaf node. The most
     * common tree policy is UCB1 but a Uniform policy may also be used. */
    class TreePolicy {
        //----typedefs/classes----//

        //----members----//
    protected:
        std::shared_ptr<Environment> environment;
        graph_t & graph;
        node_info_map_t & node_info_map;
        mcts_node_info_map_t & mcts_node_info_map;
        //----methods----//
    public:
        TreePolicy(std::shared_ptr<Environment> environment,
                   graph_t & graph,
                   node_info_map_t & node_info_map,
                   mcts_node_info_map_t & mcts_node_info_map);
        virtual node_t next(const node_t & n) = 0;
    };

    /**
     * Sample actions uniformly from available action nodes. */
    class Uniform: public TreePolicy {
        //----methods----//
    public:
        Uniform(std::shared_ptr<Environment> environment,
                graph_t & graph,
                node_info_map_t & node_info_map,
                mcts_node_info_map_t & mcts_node_info_map);
        node_t next(const node_t & node) override;
    };

} // end namespace tree_policy

#endif /* TREEPOLICY_H_ */
