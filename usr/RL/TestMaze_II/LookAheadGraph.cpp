/*
 * LookAheadGraph.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: robert
 */

#include <float.h>

#include <lemon/graph_to_eps.h>

#include "LookAheadGraph.h"

#define DEBUG_LEVEL 0
#include "debug.h"

using lemon::INVALID;

LookAheadGraph::LookAheadGraph(const double& d):
        node_info_map(graph),
        discount(d)
{}

LookAheadGraph::~LookAheadGraph() {}

void LookAheadGraph::set_discount(const double& d) {
    discount = d;
}

void LookAheadGraph::clear_tree() {
    graph.clear();
}

LookAheadGraph::value_t LookAheadGraph::get_max_action_value(const node_t& node) const {
    value_t max_action_value = -DBL_MAX;
    for(graph_t::OutArcIt out_arc(graph,node); out_arc!=INVALID; ++out_arc) {
        node_t action_node = graph.target(out_arc);
        if(node_info_map[action_node].type != ACTION) { DEBUG_OUT(0,"Error: Wrong node type."); }
        value_t action_value = node_info_map[action_node].value;
        if(action_value>max_action_value) max_action_value = action_value;
    }
    return max_action_value;
}

void LookAheadGraph::print_node(const node_t& node) const {
    DEBUG_OUT(0, "Node " << graph.id(node) << ":");
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
