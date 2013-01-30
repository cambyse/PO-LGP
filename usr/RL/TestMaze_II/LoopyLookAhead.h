/*
 * LoopyLookAhead.h
 *
 *  Created on: Jan 23, 2013
 *      Author: robert
 */

#ifndef LOOPY_LOOK_AHEAD_H_
#define LOOPY_LOOK_AHEAD_H_

#include <lemon/list_graph.h>
#include <unordered_map>

#include "Data.h"
#include "KMarkovCRF.h"
#include "Maze.h"
#include "KMDPState.h"

/*! \brief Look Ahead Tree Search using a predictive model.
 */

class LoopyLookAhead {

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

    typedef Maze predictive_model_t;

    typedef std::vector<node_t> node_vector_t;

    class StateHasher {
    public:
        size_t operator()(const graph_state_t& state) const {
            return Data::idx_from_k_mdp_state(state.get_k_mdp_state());
        }
    };
    typedef std::unordered_map<graph_state_t,node_t,StateHasher> state_node_map_t;

    LoopyLookAhead(const double& d, const bool& global = false);
    virtual ~LoopyLookAhead();

    void build_tree(const graph_state_t& s0, const size_t& depth, const predictive_model_t& model);

    value_t root_state_value() const;
    value_t action_value(const action_t& action) const;
    action_t best_action() const;

    void print_tree(const size_t& depth, const bool& eps_export = false) const;

    void set_discount(const double& d);

    void clear_tree();

protected:

    bool global_state_search;
    graph_t tree;
    node_info_map_t node_info_map;
    state_node_map_t state_node_map;
    node_t root_node;
    double discount;

    void print_node(const node_t& node) const;
    void add_subtree(const node_t& n0, const size_t& depth, const predictive_model_t& model);
};

#endif /* LOOPY_LOOK_AHEAD_H_ */
