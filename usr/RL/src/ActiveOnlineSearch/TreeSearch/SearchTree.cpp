#include "SearchTree.h"

#include "NodeFinder.h"

#include <algorithm> // for std::max
#include <queue> // std::queue
#include <sstream> // std::stringstream

#include <lemon/bfs.h>
#include <lemon/adaptors.h>

#include <QFile>
#include <QTextStream>

#include "../graph_util.h"

#include <util/util.h>
#include <util/graph_plotting.h>
#include <util/QtUtil.h>
#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::list;
using std::queue;
using std::get;
using std::tuple;
using std::make_tuple;
using std::make_pair;
using lemon::INVALID;
using util::random_select;

typedef SearchTree::node_t node_t;
typedef SearchTree::arc_t arc_t;

SearchTree::SearchTree(std::shared_ptr<AbstractEnvironment> environment,
                       double discount,
                       std::shared_ptr<node_finder::NodeFinder> node_finder):
    AbstractSearchTree(environment,discount),
    node_finder(node_finder),
    node_info_map(graph)
{
    node_finder->init(graph,node_info_map);
}

void SearchTree::init() {
    graph.clear();
    root_node = graph.addNode();
    node_info_map[root_node].type = OBSERVATION_NODE;
    node_info_map[root_node].observation = observation_handle_t();
    node_finder->init(graph,node_info_map);
    node_finder->add_observation_node(root_node);
}

void SearchTree::next() {
    // check is graph was initialized
    if(node_it_t(graph)==INVALID) {
        init();
    }
    // call actual function
    next_do();
}


void SearchTree::prune(const action_handle_t & action,
                       const observation_handle_t & observation) {
    //--------------------//
    // find new root node //
    //--------------------//

    node_t new_root_node = INVALID;
    for(out_arc_it_t arc(graph, root_node); arc!=INVALID && new_root_node==INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        if(*(node_info_map[action_node].action)==*action) {
            for(out_arc_it_t arc(graph, action_node); arc!=INVALID && new_root_node==INVALID; ++arc) {
                node_t observation_node = graph.target(arc);
                if(*(node_info_map[observation_node].observation)==*observation) {
                    new_root_node = observation_node;
                }
            }
        } else {
            DEBUG_EXPECT(0,new_root_node==INVALID);
        }
    }

    // check if new root node was found
    if(new_root_node==INVALID) {
        DEBUG_ERROR("No branch for (action --> observation) = ("
                    << *action << " --> " << *observation << ")");
        init();
        return;
    }

    //------------//
    // prune tree //
    //------------//

    /* This alternative approach only searches through the parts of the graph
     * that are close to the root. This might be less efficient for very small
     * graphs but should be much more efficient for large graphs. */

    DEBUG_OUT(2,"Pruning graph...");

    // add all children of the old root node to list of nodes to_be_processed
    queue<node_t> to_be_processed;
    for(out_arc_it_t arc(graph,root_node); arc!=INVALID; ++arc) {
        node_t node = graph.target(arc);
        DEBUG_OUT(3,"    add node " << graph.id(node) << " to kick-off nodes");
        to_be_processed.push(node);
    }

    // remove old root and set new root
    erase_node(root_node);
    root_node = new_root_node;

    // try to find a reverse (!) path from all nodes to_be_processed to the
    // (new) root_node and erase the ones such a path does not exist for. this
    // should be efficient since we are close to the root.
    auto reverse_graph = reverseDigraph(graph);
    while(!to_be_processed.empty()) {
        node_t node = to_be_processed.front();
        to_be_processed.pop();
        DEBUG_OUT(3,"    checking node " << graph.id(node));
        if(graph.valid(node) && !bfs(reverse_graph).run(node,root_node)) {
            // no path to root node: erase from graph and add children to nodes
            // to_be_processed
            DEBUG_OUT(3,"        didn't find path to root");
            for(out_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
                DEBUG_OUT(4,"            adding child node " << graph.id(graph.target(arc)));
                to_be_processed.push(graph.target(arc));
            }
            DEBUG_OUT(3,"        erasing node " << graph.id(node));
            erase_node(node);
        } else {
            IF_DEBUG(3) {
                if(graph.valid(node)) {
                    DEBUG_OUT(3,"        found path to root");
                } else {
                    DEBUG_OUT(3,"        already erased, skipping node");
                }
            }
        }
    }
}

void SearchTree::plot_graph(const char* file_name,
                            const char* command,
                            const char* parameters) const {
    graph_t::NodeMap<QString> node_map(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        node_map[node] = QString("shape=%2 label=<%3>").
            arg(node_info_map[node].type==OBSERVATION_NODE?"square":"circle").
            arg(str_html(node));
    }

    graph_t::ArcMap<QString> arc_map(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        if(node_info_map[graph.source(arc)].type==OBSERVATION_NODE) {
            arc_map[arc] = QString("style=dashed");
        } else {
            arc_map[arc] = QString("style=solid");
        }
    }

    util::plot_graph(file_name,
                     graph,
                     "style=filled truecolor=true",
                     &node_map,
                     "",
                     &arc_map,
                     command,
                     parameters);
}

const SearchTree::graph_t & SearchTree::get_graph() const {
    return graph;
}

const SearchTree::node_info_map_t & SearchTree::get_node_info_map() const {
    return node_info_map;
}

QString SearchTree::str_html(const node_t & n) const {
    QString label;
    std::stringstream s;
    if(n==root_node && node_info_map[n].observation==nullptr) {
        label = "root";
    } else if(node_info_map[n].type==OBSERVATION_NODE) {
        s << *(node_info_map[n].observation);
        label = s.str().c_str();
    } else {
        s << *(node_info_map[n].action);
        label = s.str().c_str();
    }
    return label;
}

SearchTree::arc_node_t SearchTree::find_or_create_observation_node(const node_t & action_node,
                                                                   const observation_handle_t & observation) {
    DEBUG_OUT(1,"find_or_create_observation_node()");
    DEBUG_OUT(2,"    action_node: " << graph.id(action_node));
    DEBUG_OUT(2,"    observation: " << *observation);
    using namespace return_tuple;
    node_t observation_node;
    arc_t to_observation_arc;
    bool new_arc, new_node;
    t(to_observation_arc, observation_node, new_arc, new_node)
        = node_finder->find_observation_node(action_node, observation);
    if(observation_node==INVALID) {
        // observation node doesn't exist --> create
        DEBUG_OUT(2,"    node and arc both don't exist");
        return add_observation_node(observation, action_node);
    } else if(to_observation_arc==INVALID) {
        // observation node DOES exist but not the connecting arc --> add arc
        DEBUG_OUT(2,"    node DOES exist, arc doesn't exist");
        arc_t arc = graph.addArc(action_node, observation_node);
        return arc_node_t(arc, observation_node, true, false);
    } else if(observation_node!=INVALID && to_observation_arc!=INVALID) {
        // observation node AND connecting arc were found --> just return them
        DEBUG_OUT(2,"    node and arc exist");
        DEBUG_EXPECT(0,!new_arc && !new_node);
        return arc_node_t(to_observation_arc, observation_node, new_arc, new_node);
    } else {
        DEBUG_DEAD_LINE;
        return arc_node_t(INVALID,INVALID,false,false);
    }
}

SearchTree::arc_node_t SearchTree::find_or_create_action_node(const node_t & observation_node,
                                                              const action_handle_t & action) {
    using namespace return_tuple;
    node_t action_node;
    arc_t to_action_arc;
    bool new_arc, new_node;
    t(to_action_arc, action_node, new_arc, new_node)
        = node_finder->find_action_node(observation_node, action);
    if(action_node==INVALID) {
        // action node doesn't exist --> create
        return add_action_node(action, observation_node);
    } else if(to_action_arc==INVALID) {
        // action node DOES exist but not the connecting arc --> add arc
        arc_t arc = graph.addArc(observation_node, action_node);
        return arc_node_t(arc, observation_node, true, false);
    } else if(action_node!=INVALID && to_action_arc!=INVALID) {
        // action node AND connecting arc were found --> just return them
        DEBUG_EXPECT(0,!new_arc && !new_node);
        return arc_node_t(to_action_arc, action_node, new_arc, new_node);
    } else {
        DEBUG_DEAD_LINE;
        return arc_node_t(INVALID,INVALID,false,false);
    }
}

SearchTree::arc_node_t SearchTree::add_observation_node(observation_handle_t observation,
                                                        node_t action_node) {
    node_t observation_node = graph.addNode();
    arc_t observation_arc = graph.addArc(action_node, observation_node);
    node_info_map[observation_node].type = OBSERVATION_NODE;
    node_info_map[observation_node].observation = observation;
    node_finder->add_observation_node(observation_node);
    DEBUG_OUT(3,"    adding observation node (" << graph.id(observation_node) << "): " << *observation);
    return arc_node_t(observation_arc, observation_node,true,true);
}

SearchTree::arc_node_t SearchTree::add_action_node(action_handle_t action, node_t observation_node) {
    node_t action_node = graph.addNode();
    arc_t action_arc = graph.addArc(observation_node, action_node);
    node_info_map[action_node].type = ACTION_NODE;
    node_info_map[action_node].action = action;
    node_finder->add_action_node(action_node);
    DEBUG_OUT(3,"    adding action node (" << graph.id(action_node) << "): " << *action);
    return arc_node_t(action_arc, action_node,true,true);
}

void SearchTree::erase_node(node_t node) {
    if(node_info_map[node].type==ACTION_NODE) {
        node_finder->erase_action_node(node);
    } else {
        node_finder->erase_observation_node(node);
    }
    graph.erase(node);
}
