#include "SearchTree.h"

#include <algorithm> // for std::max
#include <set>

#include <lemon/dfs.h>

#include <QFile>
#include <QTextStream>

#include <util/util.h>
#include <util/graph_plotting.h>
#include <util/QtUtil.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::list;
using std::set;
using std::get;
using std::tuple;
using std::make_tuple;
using std::make_tuple;
using lemon::INVALID;
using util::random_select;

SearchTree::SearchTree(const state_t & root_state,
                       Environment & environment,
                       double discount,
                       GRAPH_TYPE graph_type):
    node_info_map(graph),
    discount(discount),
    environment(environment),
    graph_type(graph_type)
{
    init(root_state);
}

void SearchTree::init(const state_t & s) {
    graph.clear();
    root_node = graph.addNode();
    node_info_map[root_node].state=s;
}

void SearchTree::prune(const action_t & action, const state_t & state) {
    //--------------------//
    // find new root node //
    //--------------------//

    node_t new_root_node = INVALID;
    for(out_arc_it_t arc(graph, root_node); arc!=INVALID && new_root_node==INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        if(node_info_map[action_node].action==action) {
            for(out_arc_it_t arc(graph, action_node); arc!=INVALID && new_root_node==INVALID; ++arc) {
                node_t state_node = graph.target(arc);
                if(node_info_map[state_node].state==state) {
                    new_root_node = state_node;
                }
            }
        } else {
            DEBUG_EXPECT(0,new_root_node==INVALID);
        }
    }

    // check if new root node was found
    if(new_root_node==INVALID) {
        DEBUG_OUT(1, "No branch for (state --> action --> state) = ("
                  << node_info_map[root_node].state << " --> " << action << " --> " << state << ")");
        graph.clear();
        root_node = graph.addNode();
        node_info_map[root_node].state=state;
        return;
    }

    //------------//
    // prune tree //
    //------------//

    // set new root node and remove incoming arcs from new root node to split
    // graph in two connected components
    root_node = new_root_node;
    vector<arc_t> arcs_to_erase;
    for(in_arc_it_t arc(graph, new_root_node); arc!=INVALID; ++arc) {
        arcs_to_erase.push_back(arc);
    }
    for(auto arc : arcs_to_erase) {
        graph.erase(arc);
    }

    // find nodes that are reachable from new root node
    graph_t::NodeMap<bool> reached(graph);
    lemon::dfs(graph).reachedMap(reached).run(root_node);

    // erase those that cannot be reached (but don't iterate AND erase at the
    // same time)
    vector<node_t> nodes_to_erase;
    for(node_it_t node(graph); node!=INVALID; ++node) {
        if(!reached[node]) nodes_to_erase.push_back(node);
    }
    for(node_t node : nodes_to_erase) {
        graph.erase(node);
    }
}

void SearchTree::toPdf(const char* file_name) const {

    graph_t::NodeMap<QString> node_map(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        node_map[node] = QString("shape=%2 label=<%3>").
            arg(node_info_map[node].type==STATE_NODE?"square":"circle").
            arg(str_rich(node));
    }

    graph_t::ArcMap<QString> arc_map(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        if(node_info_map[graph.source(arc)].type==STATE_NODE) {
            arc_map[arc] = QString("style=dashed");
        } else {
            arc_map[arc] = QString("style=solid");
        }
    }

    util::graph_to_pdf(file_name,
                       graph,
                       "style=filled truecolor=true",
                       &node_map,
                       "",
                       &arc_map);
}

size_t SearchTree::number_of_children(const node_t & state_node) const {
    size_t action_node_counter = 0;
    for(out_arc_it_t arc(graph, state_node); arc!=INVALID; ++arc, ++action_node_counter){}
    return action_node_counter;
}

bool SearchTree::is_fully_expanded(const node_t & state_node) const {
    DEBUG_EXPECT(1,node_info_map[state_node].type==STATE_NODE);
    return number_of_children(state_node)==environment.actions.size();
}

bool SearchTree::is_partially_expanded(const node_t & state_node) const {
    DEBUG_EXPECT(1,node_info_map[state_node].type==STATE_NODE);
    size_t action_node_counter = number_of_children(state_node);
    return action_node_counter>0 && action_node_counter<environment.actions.size();
}

bool SearchTree::is_not_expanded(const node_t & state_node) const {
    DEBUG_EXPECT(1,node_info_map[state_node].type==STATE_NODE);
    return out_arc_it_t(graph,state_node)==INVALID;
}


QString SearchTree::str(const node_t & n) const {
    bool is_state_node = node_info_map[n].type==STATE_NODE;
    return QString("%1 (%2)").
        arg(is_state_node?"STATE":"ACTION").
        arg(is_state_node?node_info_map[n].state:node_info_map[n].action);
}

QString SearchTree::str_rich(const node_t & n) const {
    bool is_state_node = node_info_map[n].type==STATE_NODE;
    // return QString("<i>%1</i>=%2").
    //     arg(is_state_node?"s":"a").
    //     arg(is_state_node?node_info_map[n].state:node_info_map[n].action);
    return QString("%1").arg(is_state_node?environment.state_name(node_info_map[n].state):environment.action_name(node_info_map[n].action));
}

double SearchTree::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}

tuple<SearchTree::arc_t,SearchTree::node_t> SearchTree::find_state_node(const node_t & action_node,
                                                                        const state_t & state) {
    DEBUG_OUT(3,"Find state node (action node (" << graph.id(action_node) << "): " <<
              environment.action_name(node_info_map[action_node].action) << ", state: " <<
              environment.state_name(state) << ")");
    switch(graph_type) {
    case TREE:
        // just look through children of this action node
        DEBUG_OUT(4,"    (TREE mode)");
        for(out_arc_it_t arc(graph, action_node); arc!=lemon::INVALID; ++arc) {
            node_t state_node = graph.target(arc);
            if(node_info_map[state_node].state==state) {
                return make_tuple(arc, state_node);
            }
        }
        break;
    case PARTIAL_DAG: {
        // first look through children of this action node
        DEBUG_OUT(4,"    (TREE mode)");
        for(out_arc_it_t arc(graph, action_node); arc!=lemon::INVALID; ++arc) {
            node_t state_node = graph.target(arc);
            if(node_info_map[state_node].state==state) {
                return make_tuple(arc, state_node);
            }
        }
        // then look through all children of this action nodes its siblings
        DEBUG_OUT(4,"    (PARTIAL_DAG mode)");
        node_t parent_node = graph.source(in_arc_it_t(graph,action_node));
        for(out_arc_it_t to_action_arc(graph,parent_node); to_action_arc!=INVALID; ++to_action_arc) {
            node_t sibling_node = graph.target(to_action_arc);
            if(sibling_node==action_node) continue;
            for(out_arc_it_t to_state_arc(graph, sibling_node); to_state_arc!=lemon::INVALID; ++to_state_arc) {
                node_t state_node = graph.target(to_state_arc);
                if(node_info_map[state_node].state==state) {
                    if(node_info_map[sibling_node].action==node_info_map[action_node].action) {
                        DEBUG_OUT(5,"    Found existing arc from " <<
                                  graph.id(action_node) << " to " << graph.id(state_node));
                        return make_tuple(to_state_arc, state_node);
                    } else {
                        DEBUG_OUT(5,"    Creating new arc from " <<
                                  graph.id(action_node) << " to " << graph.id(state_node) <<
                                  " (sibling " << graph.id(sibling_node) << ")");
                        arc_t new_arc = graph.addArc(action_node, state_node);
                        return make_tuple(new_arc, state_node);
                    }
                }
            }
        }
        break;
    }
    case FULL_DAG:
        DEBUG_OUT(4,"    (FULL_DAG mode)");
    default:
        DEBUG_DEAD_LINE;
    }

    // state node doesn't exist --> create
    node_t state_node = graph.addNode();
    node_info_map[state_node].type = STATE_NODE;
    node_info_map[state_node].state = state;
    arc_t state_arc = graph.addArc(action_node, state_node);
    DEBUG_OUT(3,"    adding state node (" << graph.id(state_node) << "): " <<
              environment.state_name(state));
    return make_tuple(state_arc, state_node);
}

tuple<SearchTree::arc_t,SearchTree::node_t> SearchTree::find_action_node(const node_t & state_node,
                                                                         const action_t & action) {
    for(out_arc_it_t arc(graph, state_node); arc!=INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        if(node_info_map[action_node].action==action) {
            return make_tuple(arc, action_node);
        }
    }
    // action node doesn't exist --> create
    node_t action_node = graph.addNode();
    node_info_map[action_node].type = ACTION_NODE;
    node_info_map[action_node].action = action;
    arc_t action_arc = graph.addArc(state_node, action_node);
    DEBUG_OUT(3,"    adding action node (" << graph.id(action_node) << "): " <<
              environment.action_name(action));
    return make_tuple(action_arc, action_node);
}
