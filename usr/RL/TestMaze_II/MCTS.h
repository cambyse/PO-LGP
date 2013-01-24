/*
 * MCTS.h
 *
 *  Created on: Jan 23, 2013
 *      Author: robert
 */

#ifndef MCTS_H_
#define MCTS_H_

#include <lemon/list_graph.h>
#include <map>

#include "Data.h"
#include "KMarkovCRF.h"
#include "Maze.h"
#include "KMDPState.h"

/*! \brief Monte Carlo Tree Search using a predictive model.
 */

class MCTS {

public:

    typedef Data::size_t        size_t;
    typedef Data::idx_t         idx_t;
    typedef Data::state_t       state_t;
    typedef Data::action_t      action_t;
    typedef Data::reward_t      reward_t;
    typedef Data::probability_t probability_t;

    typedef Data::k_mdp_state_t graph_state_t;
    typedef Data::action_t      graph_action_t;

    enum NODE_TYPE { NONE, STATE, ACTION };

    struct NodeInfo {
        NodeInfo(NODE_TYPE t = NONE): type(t), counter(0), reward_sum(0), value(0), state(), action() {}
        NODE_TYPE type;
        size_t counter;
        Data::reward_t reward_sum;
        Data::value_t value;
        KMDPState state;
        action_t action;
    };

    typedef lemon::ListDigraph graph_t;
    typedef graph_t::Node node_t;
    typedef graph_t::Arc arc_t;
    typedef graph_t::NodeMap<NodeInfo> node_info_map_t;

    typedef Maze predictive_model_t;

    typedef std::map<graph_state_t,node_t> state_to_node_map_t;
    typedef std::map<graph_action_t,node_t> action_to_node_map_t;
    typedef std::vector<node_t> node_vector;

    MCTS();
    virtual ~MCTS();

    void build_tree(const graph_state_t& s0, const size_t& depth, const size_t& sample_size, const predictive_model_t& model);
    void add_subtree(const node_t& n0, const size_t& depth, const size_t& sample_size, const predictive_model_t& model);
    void clear_tree();
    void print_tree(const size_t& depth);
    void print_node(const node_t& node);

protected:
    graph_t tree;
    node_info_map_t node_info_map;
    state_to_node_map_t state_to_node_map;
    action_to_node_map_t action_to_node_map;
    node_t root_node;

};

#endif /* MCTS_H_ */
