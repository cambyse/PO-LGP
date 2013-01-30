/*
 * LookAheadGraph.h
 *
 *  Created on: Jan 23, 2013
 *      Author: robert
 */

#ifndef LOOK_AHEAD_GRAPH_H_
#define LOOK_AHEAD_GRAPH_H_

#include <lemon/list_graph.h>

#include "Data.h"
#include "KMarkovCRF.h"
#include "KMDPState.h"

/*! \brief Base Class for look-ahead planning.
 */

class LookAheadGraph {

public:

    typedef Data::size_t        size_t;
    typedef Data::idx_t         idx_t;
    typedef Data::action_t      action_t;
    typedef Data::action_idx_t  action_idx_t;
    typedef Data::state_t       state_t;
    typedef Data::state_idx_t   state_idx_t;
    typedef Data::reward_t      reward_t;
    typedef Data::reward_idx_t  reward_idx_t;
    typedef Data::value_t       value_t;
    typedef Data::probability_t probability_t;

    typedef KMDPState graph_state_t;

    enum NODE_TYPE { NONE, STATE, ACTION };

    struct NodeInfo {
        NodeInfo():
            type(NONE),
            expected_reward(0),
            value(0),
            state(),
            action(Data::NUMBER_OF_ACTIONS) {}
        NODE_TYPE type;
        Data::reward_t expected_reward;
        Data::value_t value;
        graph_state_t state;
        action_t action;
    };


    typedef lemon::ListDigraph graph_t;
    typedef graph_t::Node node_t;
    typedef graph_t::Arc arc_t;
    typedef graph_t::NodeMap<NodeInfo> node_info_map_t;

    typedef std::vector<node_t> node_vector_t;

    LookAheadGraph(const double& d);
    virtual ~LookAheadGraph();

    virtual value_t get_action_value(const action_t& action) const = 0;
    virtual action_t get_best_action(const bool& random_tie_break) const = 0;

    void set_discount(const double& d);

    virtual void clear_tree();

protected:

    graph_t graph;
    node_t root_node;
    node_info_map_t node_info_map;
    double discount;

    virtual value_t get_max_action_value(const node_t& node) const;

    virtual void print_node(const node_t& node) const;

};

#endif /* LOOK_AHEAD_GRAPH_H_ */
