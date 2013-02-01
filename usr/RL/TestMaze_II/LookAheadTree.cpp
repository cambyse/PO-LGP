/*
 * LookAheadTree.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: robert
 */

#include <lemon/graph_to_eps.h>

#include "LookAheadTree.h"

#define DEBUG_LEVEL 0
#include "debug.h"

using lemon::INVALID;

LookAheadTree::LookAheadTree(const double& d): LookAheadGraph(d) {}

LookAheadTree::~LookAheadTree() {}

LookAheadTree::value_t LookAheadTree::root_state_value() const {
    return node_info_map[root_node].value;
}

LookAheadTree::value_t LookAheadTree::get_action_value(const action_t& action) const {
    for(graph_t::OutArcIt out_arc(graph,root_node); out_arc!=INVALID; ++out_arc) {
        node_t node = graph.target(out_arc);
        if(node_info_map[node].action==action) {
            return node_info_map[node].value;
        }
    }
    DEBUG_OUT(0,"Error: Action not found");
    return 0;
}

LookAheadTree::action_t LookAheadTree::get_best_action(const bool& random_tie_break) const {
    value_t max_action_value = -DBL_MAX;
    std::vector<action_t> actions;
    action_t best_action = Data::NUMBER_OF_ACTIONS;

    for(graph_t::OutArcIt out_arc(graph,root_node); out_arc!=INVALID; ++out_arc) {
        node_t node = graph.target(out_arc);
        value_t action_value = node_info_map[node].value;

        if(random_tie_break) {
            if(action_value>max_action_value) {
                max_action_value = action_value;
                actions.clear();
                actions.push_back(node_info_map[node].action);
            } else if(action_value==max_action_value) {
                actions.push_back(node_info_map[node].action);
            }
        } else {
            if(action_value>max_action_value) {
                max_action_value = action_value;
                best_action = node_info_map[node].action;
            }
        }

    }

    if(random_tie_break) {
        if(actions.size()==0 || max_action_value==-DBL_MAX) {
            DEBUG_OUT(0,"Error: Something went wrong (line " << __LINE__ << ")");
        }
        int action_idx = rand()%actions.size();
        DEBUG_OUT(0,"Best action: " << Data::action_strings[action_idx]);
        return actions[action_idx];
    } else {
        return best_action;
    }
}

void LookAheadTree::print_tree(const size_t& depth, const bool& eps_export) const {

    if(DEBUG_LEVEL>=2) {
        DEBUG_OUT(2,"Print all nodes:");
        for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {
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
    graph_t::NodeMap<Point> coords(graph);
    graph_t::NodeMap<double> sizes(graph);
    graph_t::NodeMap<std::string> lables(graph);
    graph_t::NodeMap<int> shapes(graph);
    graph_t::ArcMap<double> widths(graph);
    for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {
        sizes[node] = 0.9;
        lables[node] = node_info_map[node].state.print();
        if(node_info_map[node].type==STATE) {
            shapes[node] = 0;
        } else {
            shapes[node] = 2;
        }
    }
    for(graph_t::ArcIt arc(graph); arc!=INVALID; ++arc) {
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

            for(graph_t::OutArcIt out_state_arc(graph,current_state_node); out_state_arc!=INVALID; ++out_state_arc) {
                node_t current_action_node = graph.target(out_state_arc);
                if(node_info_map[current_action_node].type!=ACTION) DEBUG_OUT(0,"Error: non-ACTION node in action layer");

                print_node(current_action_node);
                coords[current_action_node] = Point(state_x_coord-0.4+0.2*action_counter, state_y_coord+y_level);
                ++action_counter;
                y_level+=0.1;

                for(graph_t::OutArcIt out_action_arc(graph,current_action_node); out_action_arc!=INVALID; ++out_action_arc) {
                    node_t next_state_node = graph.target(out_action_arc);
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
        lemon::graphToEps(graph, "look_ahead_tree.eps")
            .coords(coords)
            .title("Look Ahead Tree")
            .absoluteNodeSizes().nodeScale(0.1).nodeSizes(sizes)
            .nodeShapes(shapes)
            .nodeTexts(lables).nodeTextSize(0.01)
            .absoluteArcWidths().arcWidthScale(.1).arcWidths(widths)
            .run();
    }
}
