/*
 * LookAheadTree.h
 *
 *  Created on: Jan 23, 2013
 *      Author: robert
 */

#ifndef LOOK_AHEAD_TREE_H_
#define LOOK_AHEAD_TREE_H_

#include "LookAheadGraph.h"

#include <float.h>

#define DEBUG_LEVEL 0
#include "debug.h"

/*! \brief Look Ahead Tree Search using a predictive model.
 */

class LookAheadTree: public LookAheadGraph {

public:

    LookAheadTree(const double& d);
    virtual ~LookAheadTree();

    template < class Model >
    void build_tree(
            const graph_state_t& s0,
            const size_t& depth,
            const Model& model,
            probability_t(Model::*prediction)(const Data::k_mdp_state_t&, const action_t&, const state_t&, const reward_t&) const
    );

    value_t root_state_value() const;
    virtual value_t get_action_value(const action_t& action) const;
    virtual action_t get_best_action(const bool& random_tie_break = true) const;

    void print_tree(const size_t& depth, const bool& eps_export = false) const;

protected:

    template < class Model >
    void add_subtree(
            const node_t& n0,
            const size_t& depth,
            const Model& model,
            probability_t(Model::*prediction)(const Data::k_mdp_state_t&, const action_t&, const state_t&, const reward_t&) const
    );
};

//===========================================================================================//
//                                  Function implementations                                 //
//===========================================================================================//

template < class Model >
void LookAheadTree::build_tree(
        const graph_state_t& s0,
        const size_t& depth,
        const Model& model,
        probability_t(Model::*prediction)(const Data::k_mdp_state_t&, const action_t&, const state_t&, const reward_t&) const
) {

    // prepare empty tree
    clear_tree();
    root_node = graph.addNode();

    // set node info
    node_info_map[root_node].type = STATE;
    node_info_map[root_node].state = s0;
    node_info_map[root_node].expected_reward = 0;
    node_info_map[root_node].value = 0;

    // create subtree
    if(depth>0) {
        DEBUG_OUT(1,"Building tree:");
        DEBUG_OUT(1,"    root state " << s0.print());
        DEBUG_OUT(1,"    depth      " << depth);
        add_subtree(root_node, depth, model, prediction);
    }

    // update root state node
    node_info_map[root_node].value = discount*get_max_action_value(root_node);
}

template < class Model >
void LookAheadTree::add_subtree(
        const node_t& n0,
        const size_t& depth,
        const Model& model,
        probability_t(Model::*prediction)(const Data::k_mdp_state_t&, const action_t&, const state_t&, const reward_t&) const
) {

    //-------------------------//
    // check for maximum depth //
    //-------------------------//

    if(depth==0) {
        node_info_map[n0].value = node_info_map[n0].expected_reward;
        DEBUG_OUT(1,"Max depth reached");
        return;
    }
    if(graph_t::OutArcIt(graph,n0) != lemon::INVALID) {
        DEBUG_OUT(0,"Error: State node processed more than once (already has outgoing actions)");
        return;
    }

    //------------------------//
    // go through all actions //
    //------------------------//

    DEBUG_OUT(1,"Determine possible transitions from state " << node_info_map[n0].state.print());

    for(action_idx_t action_idx = 0; action_idx<Data::action_n; ++action_idx) {

        action_t action = Data::action_from_idx(action_idx);

        //--------------------//
        // create action node //
        //--------------------//

        node_t action_node = graph.addNode();
        node_info_map[action_node].type = ACTION;
        node_info_map[action_node].action = action;
        node_info_map[action_node].state = node_info_map[n0].state;
        node_info_map[action_node].value = 0;
        graph.addArc(n0, action_node);

        //------------------------------------------------//
        // determine reachable states and expected reward //
        //------------------------------------------------//

        DEBUG_OUT(2,"action = " << Data::action_strings[action_idx]);

        graph_state_t new_state;

        for(state_idx_t state_idx=0; state_idx<Data::state_n; ++state_idx) {
            state_t state = Data::state_from_idx(state_idx);

            for(reward_idx_t reward_idx=0; reward_idx<Data::reward_n; ++reward_idx) {
                reward_t reward = Data::reward_from_idx(reward_idx);
                new_state = node_info_map[n0].state;
                new_state.new_state(action,state,reward);
                probability_t prob = (model.*prediction)(node_info_map[action_node].state.get_k_mdp_state(), action, state, reward);

                if(prob==0) continue;

                // find node or create new one
                node_t state_node = graph.addNode();
                graph.addArc(action_node, state_node);
                node_info_map[state_node].type = STATE;
                node_info_map[state_node].state = new_state;
                node_info_map[state_node].expected_reward = reward;

                // add subtree
                add_subtree(state_node, depth-1, model, prediction);

                // update action value
                node_info_map[action_node].value += prob*node_info_map[state_node].value;

            }
        }
    }

    // update state value
    node_info_map[n0].value = -DBL_MAX;
    for(graph_t::OutArcIt out_arc(graph,n0); out_arc!=lemon::INVALID; ++out_arc) {
        node_t action_node = graph.target(out_arc);
        if(node_info_map[action_node].value>node_info_map[n0].value) {
            node_info_map[n0].value = node_info_map[action_node].value;
        }
    }
    node_info_map[n0].value *= discount;
    node_info_map[n0].value += node_info_map[n0].expected_reward;
}

#include "debug_exclude.h"

#endif /* LOOK_AHEAD_TREE_H_ */
