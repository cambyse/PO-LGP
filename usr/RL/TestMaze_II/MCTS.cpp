/*
 * MCTS.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: robert
 */

#include <float.h>

#include "MCTS.h"

#define DEBUG_LEVEL 0
#include "debug.h"

using lemon::INVALID;

MCTS::MCTS(const double& d):
    node_info_map(tree),
    discount(d)
{}

MCTS::~MCTS() {}

void MCTS::build_tree(const graph_state_t& s0, const size_t& depth, const predictive_model_t& model, const size_t& sample_size) {

    // prepare empty tree
    clear_tree();
    root_node = tree.addNode();

    // set node info
    node_info_map[root_node].type = STATE;
    node_info_map[root_node].state = s0;
    node_info_map[root_node].counter = 1;
    node_info_map[root_node].reward_sum = 0;

    // create subtree
    if(depth>0) {
        DEBUG_OUT(1,"Building tree:");
        DEBUG_OUT(1,"    root state " << s0.print());
        DEBUG_OUT(1,"    depth      " << depth);
        DEBUG_OUT(1,"    samples    " << sample_size);
        add_subtree(root_node, depth, sample_size, model);
    }

    // update root state node
    value_t max_action_value = -DBL_MAX;
    for(graph_t::OutArcIt out_arc(tree,root_node); out_arc!=INVALID; ++out_arc) {
        node_t node = tree.target(out_arc);
        value_t node_val = node_info_map[node].value;
        if(node_val>max_action_value) max_action_value = node_val;
    }
    node_info_map[root_node].value = node_info_map[root_node].reward_sum/node_info_map[root_node].counter;
    node_info_map[root_node].value += discount*max_action_value;
}

MCTS::value_t MCTS::root_state_value() const {
    return node_info_map[root_node].value;
}

MCTS::value_t MCTS::action_value(const action_t& action) const {
    for(graph_t::OutArcIt out_arc(tree,root_node); out_arc!=INVALID; ++out_arc) {
        node_t node = tree.target(out_arc);
        if(node_info_map[node].action==action) {
            return node_info_map[node].value;
        }
    }
    DEBUG_OUT(0,"Error: Something went wrong");
    return 0;
}

MCTS::action_t MCTS::best_action() const {
    value_t max_action_value = -DBL_MAX;
    std::vector<action_t> actions;
    for(graph_t::OutArcIt out_arc(tree,root_node); out_arc!=INVALID; ++out_arc) {
        node_t node = tree.target(out_arc);
        value_t node_val = node_info_map[node].value;
        if(node_val>max_action_value) {
            max_action_value = node_val;
            actions.clear();
            actions.push_back(node_info_map[node].action);
        } else if(node_val==max_action_value) {
            actions.push_back(node_info_map[node].action);
        }
    }
    if(actions.size()==0|| max_action_value==-DBL_MAX) {
        DEBUG_OUT(0,"Error: Something went wrong");
    }
    int action_idx = rand()%actions.size();
    return actions[action_idx];
}

void MCTS::print_tree(const size_t& depth) const {

    if(DEBUG_LEVEL>=2) {
        DEBUG_OUT(2,"Print all nodes:");
        for(graph_t::NodeIt node(tree); node!=INVALID; ++node) {
            print_node(node);
        }
        DEBUG_OUT(2,"\n");
    }

    node_vector * current_state_layer = new node_vector();
    node_vector * next_state_layer = new node_vector();

    DEBUG_OUT(0,"Print tree:");
    current_state_layer->push_back(root_node);

    for(size_t current_depth = 0; current_depth<=depth; ++current_depth) {
        DEBUG_OUT(0,"---layer " << current_depth << "---");
        for(idx_t state_idx=0; state_idx<(idx_t)current_state_layer->size(); ++state_idx) {
            node_t current_state_node = (*current_state_layer)[state_idx];
            if(node_info_map[current_state_node].type!=STATE) DEBUG_OUT(0,"Error: non-STATE node in state layer");
            print_node(current_state_node);
            for(graph_t::OutArcIt out_state_arc(tree,current_state_node); out_state_arc!=INVALID; ++out_state_arc) {
                node_t current_action_node = tree.target(out_state_arc);
                if(node_info_map[current_action_node].type!=ACTION) DEBUG_OUT(0,"Error: non-ACTION node in action layer");
                print_node(current_action_node);
                for(graph_t::OutArcIt out_action_arc(tree,current_action_node); out_action_arc!=INVALID; ++out_action_arc) {
                    node_t next_state_node = tree.target(out_action_arc);
                    next_state_layer->push_back(next_state_node);
                }
            }
        }
        current_state_layer->clear();
        node_vector * tmp = current_state_layer;
        current_state_layer = next_state_layer;
        next_state_layer = tmp;
    }

    delete current_state_layer;
    delete next_state_layer;
}

void MCTS::set_discount(const double& d) {
    discount = d;
}

void MCTS::clear_tree() {
    tree.clear();
}

void MCTS::print_node(const node_t& node) const {
    DEBUG_OUT(0, "Node " << tree.id(node) << ":");
    switch(node_info_map[node].type) {
    case NONE:
        DEBUG_OUT(0, "    type:   " << "NONE" );
        break;
    case STATE:
        DEBUG_OUT(0, "    type:   " << "STATE" );
        DEBUG_OUT(0, "    state:  " << node_info_map[node].state.print() );
        break;
    case ACTION:
        DEBUG_OUT(0, "    type:   " << "ACTION" );
        DEBUG_OUT(0, "    state:  " << node_info_map[node].state.print() );
        DEBUG_OUT(0, "    action: " << Data::action_strings[node_info_map[node].action] );
        break;
    }
    DEBUG_OUT(0, "    counts:  " << node_info_map[node].counter );
    DEBUG_OUT(0, "    rew_sum: " << node_info_map[node].reward_sum );
    DEBUG_OUT(0, "    value:   " << node_info_map[node].value );
}

void MCTS::add_subtree(const node_t& n0, const size_t& depth, const size_t& sample_size, const predictive_model_t& model) {

    //-------------------------//
    // check for maximum depth //
    //-------------------------//

    if(depth==0) {
        DEBUG_OUT(1,"Max depth reached");
        return;
    }

    //------------------------//
    // go through all actions //
    //------------------------//

    for(Data::action_idx_t action_idx = 0; action_idx<(idx_t)Data::action_n; ++action_idx) {

        //--------------------------------//
        // find action node or create new //
        //--------------------------------//

        node_t action_node = INVALID;
        Data::action_t action = Data::action_from_idx(action_idx);

        for(graph_t::OutArcIt out_arc(tree,n0); out_arc!=INVALID; ++out_arc) { // search neighbors
            node_t node = tree.target(out_arc);
            if(node_info_map[node].action==action) {
                action_node = node;
                break;
            }
        }
        if(action_node==INVALID) { // not found within neighbors
            action_node = tree.addNode();
            node_info_map[action_node].type = ACTION;
            node_info_map[action_node].action = action;
            node_info_map[action_node].state = node_info_map[n0].state;
            tree.addArc(n0, action_node);
        }

        //---------------------------------------------//
        // perform action until sample size is reached //
        //---------------------------------------------//

        while(node_info_map[action_node].counter<sample_size) {

            // increment counter
            ++node_info_map[action_node].counter;

            // perform transition
            DEBUG_OUT(1,"Performing transition from state " << node_info_map[n0].state.print());
            DEBUG_OUT(2,"action = " << Data::action_strings[action_idx]);
            KMDPState new_state = node_info_map[n0].state;
            reward_t new_reward = Data::min_reward;
            probability_t prob_threshold = drand48();
            probability_t prob_accum = 0;
            bool found = false;
            for(Data::state_idx_t state_idx=0; state_idx<(idx_t)Data::state_n && !found; ++state_idx) {
                for(Data::reward_idx_t reward_idx=0; reward_idx<(idx_t)Data::reward_n && !found; ++reward_idx) {
                    state_t state = Data::state_from_idx(state_idx);
                    reward_t reward = Data::reward_from_idx(reward_idx);
                    DEBUG_OUT(2,"    Checking state(" << state << "), reward(" << reward << ")");
                    probability_t prob = model.get_prediction(node_info_map[action_node].state.get_k_mdp_state(), action, state, reward);
                    prob_accum += prob;
                    DEBUG_OUT(2,"        --> prob = " << prob);
                    if(prob_accum>prob_threshold){
                        found = true;
                        new_state.new_state(action, state, reward);
                        new_reward = reward;
                        DEBUG_OUT(2,"    SELECTED");
                    }
                }
            }
            if(!found || prob_accum>1) {
                if(prob_accum<1) {
                    DEBUG_OUT(0,"Error: Probabilities do not accumulate to one [sum(p)=" << prob_accum << "]");
                } else if(prob_accum>1){
                    DEBUG_OUT(0,"Error: Probabilities accumulate to larger than one [sum(p)=" << prob_accum << "]");
                } else {
                    DEBUG_OUT(0,"Error: No idea what happened?!");
                }
            }

            // try to find node for new state
            node_t state_node = INVALID;
            for(graph_t::OutArcIt out_arc(tree,action_node); out_arc!=INVALID; ++out_arc) { // search neighbors
                node_t node = tree.target(out_arc);
                if(node_info_map[node].state==new_state && node_info_map[node].type == STATE) {
                    state_node = node;
                    break;
                }
            }
            if(state_node==INVALID) {
                state_node = tree.addNode();
                tree.addArc(action_node, state_node);
                node_info_map[state_node].type = STATE;
                node_info_map[state_node].state = new_state;
            }

            // add subtree for node
            add_subtree(state_node, depth-1, sample_size, model);

            // update state node
            ++node_info_map[state_node].counter;
            node_info_map[state_node].reward_sum += new_reward;
            value_t max_action_value = -DBL_MAX;
            for(graph_t::OutArcIt out_arc(tree,state_node); out_arc!=INVALID; ++out_arc) {
                node_t node = tree.target(out_arc);
                value_t node_val = node_info_map[node].value;
                if(node_val>max_action_value) max_action_value = node_val;
            }
            if(max_action_value==-DBL_MAX) {
                if(depth>1) {
                    DEBUG_OUT(0, "Error: No action node available at non-leaf node");
                }
                max_action_value=0;
            }
            node_info_map[state_node].value = node_info_map[state_node].reward_sum/node_info_map[state_node].counter;
            node_info_map[state_node].value += discount*max_action_value;
        }
        // update action node
        node_info_map[action_node].reward_sum = 0;
        node_info_map[action_node].value = 0;
        for(graph_t::OutArcIt out_arc(tree,action_node); out_arc!=INVALID; ++out_arc) {
            node_t node = tree.target(out_arc);
            node_info_map[action_node].value += node_info_map[node].value*node_info_map[node].counter;
        }
        node_info_map[action_node].value /= node_info_map[action_node].counter;
    }
}
