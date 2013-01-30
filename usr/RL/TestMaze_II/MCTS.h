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
    typedef Data::value_t       value_t;
    typedef Data::probability_t probability_t;

    typedef KMDPState graph_state_t;

    enum NODE_TYPE { NONE, STATE, ACTION };

    struct NodeInfo {
        NodeInfo(NODE_TYPE t = NONE):
            type(t),
            counter(0),
            reward_sum(0),
            value(0),
            state(),
            action(Data::NUMBER_OF_ACTIONS) {}
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

    typedef std::vector<node_t> node_vector;

    MCTS(const double& d);
    virtual ~MCTS();

    void build_tree(const graph_state_t& s0, const size_t& depth, const predictive_model_t& model, const size_t& sample_size = 1);

    value_t root_state_value() const;
    value_t action_value(const action_t& action) const;
    action_t best_action() const;

    void print_tree(const size_t& depth) const;

    void set_discount(const double& d);

    void clear_tree();

protected:

    graph_t tree;
    node_info_map_t node_info_map;
    node_t root_node;
    double discount;

    void print_node(const node_t& node) const;
    void add_subtree(const node_t& n0, const size_t& depth, const size_t& sample_size, const predictive_model_t& model);
};

#endif /* MCTS_H_ */
