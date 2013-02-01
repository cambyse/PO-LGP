/*
 * LoopyLookAhead.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: robert
 */

#include <float.h>

#include <lemon/graph_to_eps.h>

#include "LoopyLookAhead.h"

#define DEBUG_LEVEL 0
#include "debug.h"

using lemon::INVALID;

LoopyLookAhead::LoopyLookAhead(const double& d, const bool& global):
        global_state_search(global),
        node_info_map(tree),
        discount(d)
{}

LoopyLookAhead::~LoopyLookAhead() {}

void LoopyLookAhead::build_tree(const graph_state_t& s0, const size_t& depth, const predictive_model_t& model) {

    // prepare empty tree
    clear_tree();
    root_node = tree.addNode();

    // set node info
    node_info_map[root_node].type = STATE;
    node_info_map[root_node].state = s0;
    node_info_map[root_node].expected_reward = 0;
    node_info_map[root_node].value = 0;

    // update state node map
    if(global_state_search) {
        state_node_map[s0] = root_node;
    }

    // create subtree
    if(depth>0) {
        DEBUG_OUT(1,"Building tree:");
        DEBUG_OUT(1,"    root state " << s0.print());
        DEBUG_OUT(1,"    depth      " << depth);
        add_subtree(root_node, depth, model);
    }

    // update root state node
    value_t max_action_value = -DBL_MAX;
    for(graph_t::OutArcIt out_arc(tree,root_node); out_arc!=INVALID; ++out_arc) {
        node_t action_node = tree.target(out_arc);
        if(node_info_map[action_node].type != ACTION) { DEBUG_OUT(0,"Error: Wrong node type."); }
        value_t action_value = node_info_map[action_node].value;
        if(action_value>max_action_value) max_action_value = action_value;
    }
    node_info_map[root_node].value = discount*max_action_value;
}

LoopyLookAhead::value_t LoopyLookAhead::root_state_value() const {
    return node_info_map[root_node].value;
}

LoopyLookAhead::value_t LoopyLookAhead::action_value(const action_t& action) const {
    for(graph_t::OutArcIt out_arc(tree,root_node); out_arc!=INVALID; ++out_arc) {
        node_t node = tree.target(out_arc);
        if(node_info_map[node].action==action) {
            return node_info_map[node].value;
        }
    }
    DEBUG_OUT(0,"Error: Action not found");
    return 0;
}

LoopyLookAhead::action_t LoopyLookAhead::best_action() const {
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
        DEBUG_OUT(0,"Error: Something went wrong (line " << __LINE__ << ")");
    }
    int action_idx = rand()%actions.size();
    DEBUG_OUT(0,"Best action: " << Data::action_strings[actions[action_idx]]);
    return actions[action_idx];
}

void LoopyLookAhead::print_tree(const size_t& depth, const bool& eps_export) const {

    if(DEBUG_LEVEL>=2) {
        DEBUG_OUT(2,"Print all nodes:");
        for(graph_t::NodeIt node(tree); node!=INVALID; ++node) {
            print_node(node);
        }
        DEBUG_OUT(2,"\n");
    }

    node_vector_t * current_state_layer = new node_vector_t();
    node_vector_t * next_state_layer = new node_vector_t();

    DEBUG_OUT(0,"Print tree:");
    current_state_layer->push_back(root_node);

    // for graphical output
    typedef lemon::dim2::Point<double> Point;
    graph_t::NodeMap<Point> coords(tree);
    graph_t::NodeMap<double> sizes(tree);
    graph_t::NodeMap<std::string> lables(tree);
    graph_t::NodeMap<int> shapes(tree);
    graph_t::ArcMap<double> widths(tree);
    for(graph_t::NodeIt node(tree); node!=INVALID; ++node) {
        sizes[node] = 0.9;
        lables[node] = node_info_map[node].state.print();
        if(node_info_map[node].type==STATE) {
            shapes[node] = 0;
        } else {
            shapes[node] = 2;
        }
    }
    for(graph_t::ArcIt arc(tree); arc!=INVALID; ++arc) {
        widths[arc] = 0.1;
    }

    for(size_t current_depth = 0; current_depth<=depth; ++current_depth) {

        DEBUG_OUT(0,"---layer " << current_depth << "---");
        int state_node_sign = -1;

        for(idx_t state_idx=0; state_idx<(idx_t)current_state_layer->size(); ++state_idx) {
            node_t current_state_node = (*current_state_layer)[state_idx];
            if(node_info_map[current_state_node].type!=STATE) DEBUG_OUT(0,"Error: non-STATE node in state layer");

            print_node(current_state_node);
            state_node_sign *= -1;
            double state_x_coord = state_node_sign*((state_idx+1)/2);
            double state_y_coord = current_depth;
            coords[current_state_node] = Point(state_node_sign*((state_idx+1)/2), current_depth);

            int action_counter = 0;
            double y_level = 0.1;

            for(graph_t::OutArcIt out_state_arc(tree,current_state_node); out_state_arc!=INVALID; ++out_state_arc) {
                node_t current_action_node = tree.target(out_state_arc);
                if(node_info_map[current_action_node].type!=ACTION) DEBUG_OUT(0,"Error: non-ACTION node in action layer");

                print_node(current_action_node);
                coords[current_action_node] = Point(state_x_coord-0.4+0.2*action_counter, state_y_coord+y_level);
                ++action_counter;
                y_level+=0.1;

                for(graph_t::OutArcIt out_action_arc(tree,current_action_node); out_action_arc!=INVALID; ++out_action_arc) {
                    node_t next_state_node = tree.target(out_action_arc);
                    next_state_layer->push_back(next_state_node);
                }
            }
        }
        current_state_layer->clear();
        node_vector_t * tmp = current_state_layer;
        current_state_layer = next_state_layer;
        next_state_layer = tmp;
    }

    delete current_state_layer;
    delete next_state_layer;

    if(eps_export) {
        lemon::graphToEps(tree, "look_ahead_tree.eps")
            .coords(coords)
            .title("Look Ahead Tree")
            .absoluteNodeSizes().nodeScale(0.1).nodeSizes(sizes)
            .nodeShapes(shapes)
            .nodeTexts(lables).nodeTextSize(0.01)
            .absoluteArcWidths().arcWidthScale(.1).arcWidths(widths)
            .run();
    }
}

void LoopyLookAhead::set_discount(const double& d) {
    discount = d;
}

void LoopyLookAhead::clear_tree() {
    tree.clear();
    if(global_state_search) {
        state_node_map.clear();
    }
}

void LoopyLookAhead::print_node(const node_t& node) const {
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
    DEBUG_OUT(0, "    rew_sum: " << node_info_map[node].expected_reward );
    DEBUG_OUT(0, "    value:   " << node_info_map[node].value );
}

void LoopyLookAhead::add_subtree(const node_t& n0, const size_t& depth, const predictive_model_t& model) {

    //-------------------------//
    // check for maximum depth //
    //-------------------------//

    if(depth==0) {
        node_info_map[n0].value = node_info_map[n0].expected_reward;
        DEBUG_OUT(1,"Max depth reached");
        return;
    }
    if(graph_t::OutArcIt(tree,n0) != INVALID) {
        DEBUG_OUT(0,"Error: State node processed more than once (already has outgoing actions)");
        return;
    }

    //------------------------//
    // go through all actions //
    //------------------------//

    DEBUG_OUT(1,"Determine possible transitions from state " << node_info_map[n0].state.print());

    for(action_idx_t action_idx = 0; action_idx<(idx_t)Data::action_n; ++action_idx) {

        action_t action = Data::action_from_idx(action_idx);

        //--------------------//
        // create action node //
        //--------------------//

        node_t action_node = tree.addNode();
        node_info_map[action_node].type = ACTION;
        node_info_map[action_node].action = action;
        node_info_map[action_node].state = node_info_map[n0].state;
        node_info_map[action_node].value = 0;
        tree.addArc(n0, action_node);

        //------------------------------------------------//
        // determine reachable states and expected reward //
        //------------------------------------------------//

        DEBUG_OUT(2,"action = " << Data::action_strings[action_idx]);

        graph_state_t new_state;

        for(state_idx_t state_idx=0; state_idx<(idx_t)Data::state_n; ++state_idx) {
            state_t state = Data::state_from_idx(state_idx);

            for(reward_idx_t reward_idx=0; reward_idx<(idx_t)Data::reward_n; ++reward_idx) {
                reward_t reward = Data::reward_from_idx(reward_idx);
                new_state = node_info_map[n0].state;
                new_state.new_state(action,state,reward);
                probability_t prob = model.get_prediction(node_info_map[action_node].state.get_k_mdp_state(), action, state, reward);

                if(prob==0) continue;

                // find node or create new one
                node_t state_node = INVALID;
                if(global_state_search) {
                    state_node_map_t::iterator it = state_node_map.find(new_state);
                    if(it!=state_node_map.end()) { // found globally
                        state_node = it->second;
                        tree.addArc(action_node,state_node);
                    }
                }
                if(state_node==INVALID){ // create new node
                    state_node = tree.addNode();
                    tree.addArc(action_node, state_node);
                    node_info_map[state_node].type = STATE;
                    node_info_map[state_node].state = new_state;
                    node_info_map[state_node].expected_reward = reward;

                    // update state node map
                    if(global_state_search) {
                        state_node_map[new_state] = state_node;
                    }

                    // add subtree
                    add_subtree(state_node, depth-1, model);
                }

                // update action value
                node_info_map[action_node].value += prob*node_info_map[state_node].value;

            }
        }
    }

    // update state value
    node_info_map[n0].value = -DBL_MAX;
    for(graph_t::OutArcIt out_arc(tree,n0); out_arc!=INVALID; ++out_arc) {
        node_t action_node = tree.target(out_arc);
        if(node_info_map[action_node].value>node_info_map[n0].value) {
            node_info_map[n0].value = node_info_map[action_node].value;
        }
    }
    node_info_map[n0].value *= discount;
    node_info_map[n0].value += node_info_map[n0].expected_reward;
}
