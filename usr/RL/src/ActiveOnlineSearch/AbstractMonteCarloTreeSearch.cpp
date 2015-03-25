#include "AbstractMonteCarloTreeSearch.h"

#include <float.h>

#include <util/graph_plotting.h>

using lemon::INVALID;

void AbstractMonteCarloTreeSearch::prune(const action_t & a, const state_t & s) {
    SearchTree::prune(a,s);
}

AbstractMonteCarloTreeSearch::AbstractMonteCarloTreeSearch(const state_t & root_state,
                                                           std::shared_ptr<const Environment> environment,
                                                           double discount,
                                                           GRAPH_TYPE graph_type):
    SearchTree(root_state, environment, discount, graph_type),
    graph(this->get_graph()),
    mcts_node_info_map(graph),
    mcts_arc_info_map(graph){}

void AbstractMonteCarloTreeSearch::toPdf(const char* file_name) const {

    //-----------------------------------------//
    // get min and max value/reward and counts //
    //-----------------------------------------//
    double min_val = DBL_MAX;
    double max_val = -DBL_MAX;
    int max_counts = 0;
    for(node_it_t node(graph); node!=INVALID; ++node) {
        min_val = std::min(min_val, mcts_node_info_map[node].get_value());
        max_val = std::max(max_val, mcts_node_info_map[node].get_value());
        max_counts = std::max(max_counts, mcts_node_info_map[node].get_transition_counts());
    }
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        if(type(graph.source(arc))==ACTION_NODE) {
            min_val = std::min(min_val, mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_counts());
            max_val = std::max(max_val, mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_counts());
        }
    }
    double norm = std::max(fabs(min_val), fabs(max_val));
    norm = norm==0?1:norm;

    graph_t::NodeMap<QString> node_map(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        double value = mcts_node_info_map[node].get_value();
        node_map[node] = QString("shape=%1 label=<%2<BR/>id=%5<BR/>#%3/%10<BR/>V=%4 +/- %11<BR/>R=%9> fillcolor=\"%6 %7 1\" penwidth=%8").
            arg(type(node)==STATE_NODE?"square":"circle").
            arg(str_rich(node)).
            arg(mcts_node_info_map[node].get_transition_counts()).
            arg(value,0,'g',2).
            arg(graph.id(node)).
            arg(value>0?0.3:0).
            arg(color_rescale(fabs(value/norm))).
            arg(10.*mcts_node_info_map[node].get_transition_counts()/max_counts+0.1).
            arg(mcts_node_info_map[node].get_return_sum()).
            arg(mcts_node_info_map[node].get_rollout_counts()).
            arg(sqrt(mcts_node_info_map[node].get_value_variance()/mcts_node_info_map[node].get_rollout_counts()),0,'g',2);
    }

    graph_t::ArcMap<QString> arc_map(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        node_t source = graph.source(arc);
        double value = mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_counts();
        if(type(source)==STATE_NODE) {
            arc_map[arc] = QString("style=dashed label=<#%1<BR/>r=%4> color=\"%2 %3 %3\"").
                arg(mcts_arc_info_map[arc].get_counts()).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm))).
                arg(mcts_arc_info_map[arc].get_reward_sum());
        } else {
            arc_map[arc] = QString("style=solid label=<#%2<BR/>r=%6> penwidth=%3 color=\"%4 %5 %5\"").
                arg(mcts_arc_info_map[arc].get_counts()).
                arg(5.*mcts_arc_info_map[arc].get_counts()/mcts_node_info_map[source].get_transition_counts()+0.1).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm))).
                arg(mcts_arc_info_map[arc].get_reward_sum());
        }
    }

    util::graph_to_pdf(file_name,
                       graph,
                       "style=filled truecolor=true",
                       &node_map,
                       "",
                       &arc_map);
}
