#include "AbstractMonteCarloTreeSearch.h"

#include <float.h>

#include <util/graph_plotting.h>

using lemon::INVALID;

AbstractMonteCarloTreeSearch::MCTSNodeInfo::MCTSNodeInfo():
    backtrace(lemon::INVALID, lemon::INVALID, lemon::INVALID, lemon::INVALID)
{}

void AbstractMonteCarloTreeSearch::prune(const action_t & a, const state_t & s) {
    SearchTree::prune(a,s);
    mcts_node_info_map[root()].backtrace = std::make_tuple<node_t,arc_t,node_t,arc_t>(lemon::INVALID,
                                                                                    lemon::INVALID,
                                                                                    lemon::INVALID,
                                                                                    lemon::INVALID);
}

AbstractMonteCarloTreeSearch::AbstractMonteCarloTreeSearch(const state_t & root_state,
                                                           Environment & environment,
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
        min_val = std::min(min_val, mcts_node_info_map[node].value);
        max_val = std::max(max_val, mcts_node_info_map[node].value);
        max_counts = std::max(max_counts, mcts_node_info_map[node].counts);
    }
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        if(type(graph.source(arc))==ACTION_NODE) {
            min_val = std::min(min_val, mcts_arc_info_map[arc].reward_sum/mcts_arc_info_map[arc].counts);
            max_val = std::max(max_val, mcts_arc_info_map[arc].reward_sum/mcts_arc_info_map[arc].counts);
        }
    }
    double norm = std::max(fabs(min_val), fabs(max_val));
    norm = norm==0?1:norm;

    graph_t::NodeMap<QString> node_map(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        double norm_val = mcts_node_info_map[node].value/norm;
        node_map[node] = QString("shape=%1 label=<%2<BR/>#%3<BR/>%5> fillcolor=\"%4 %5 1\" penwidth=%6").
            arg(type(node)==STATE_NODE?"square":"circle").
            arg(str_rich(node)).
            arg(mcts_node_info_map[node].counts).
            arg(norm_val>0?0.3:0).
            arg(color_rescale(fabs(norm_val))).
            arg(10.*mcts_node_info_map[node].counts/max_counts+0.1);
    }

    graph_t::ArcMap<QString> arc_map(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        node_t source = graph.source(arc);
        double norm_val = mcts_arc_info_map[arc].reward_sum/mcts_arc_info_map[arc].counts/norm;
        if(type(source)==STATE_NODE) {
            arc_map[arc] = QString("style=dashed label=<#%1> color=\"%2 %3 %3\"").
                arg(mcts_arc_info_map[arc].counts).
                arg(norm_val>0?0.3:0).
                arg(color_rescale(fabs(norm_val)));
        } else {
            arc_map[arc] = QString("style=solid label=<#%2> penwidth=%3 color=\"%4 %5 %5\"").
                arg(mcts_arc_info_map[arc].counts).
                arg(5.*mcts_arc_info_map[arc].counts/mcts_node_info_map[source].counts+0.1).
                arg(norm_val>0?0.3:0).
                arg(color_rescale(fabs(norm_val)));
        }
    }

    util::graph_to_pdf(file_name,
                       graph,
                       "style=filled truecolor=true",
                       &node_map,
                       "",
                       &arc_map,
                       false);
}
