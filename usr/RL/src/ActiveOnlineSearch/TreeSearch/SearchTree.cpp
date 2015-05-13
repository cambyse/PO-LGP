#include "SearchTree.h"

#include <algorithm> // for std::max
#include <queue> // std::queue

#include <lemon/bfs.h>
#include <lemon/adaptors.h>

#include <QFile>
#include <QTextStream>

#include "../Environment/Environment.h"

#include "../graph_util.h"

#include <util/util.h>
#include <util/graph_plotting.h>
#include <util/QtUtil.h>

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
                       GRAPH_TYPE graph_type):
    environment(environment),
    discount(discount),
    graph_type(graph_type),
    node_info_map(graph)
{}

void SearchTree::init(const observation_handle_t & o,
                      const state_handle_t & s) {
    graph.clear();
    root_node = graph.addNode();
    node_info_map[root_node].observation = o;
    root_state = s;
    if(graph_type==FULL_DAG) {
        level_map_list.assign({level_map_t({make_pair(o,root_node)})});
        node_info_map[root_node].level_map_it = level_map_list.begin();
    }
}

void SearchTree::prune(const action_handle_t & action,
                       const observation_handle_t & observation,
                       const state_handle_t & state) {
    //--------------------//
    // find new root node //
    //--------------------//

    node_t new_root_node = INVALID;
    for(out_arc_it_t arc(graph, root_node); arc!=INVALID && new_root_node==INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        if(node_info_map[action_node].action==action) {
            for(out_arc_it_t arc(graph, action_node); arc!=INVALID && new_root_node==INVALID; ++arc) {
                node_t observation_node = graph.target(arc);
                if(node_info_map[observation_node].observation==observation) {
                    new_root_node = observation_node;
                }
            }
        } else {
            DEBUG_EXPECT(0,new_root_node==INVALID);
        }
    }

    // check if new root node was found
    if(new_root_node==INVALID) {
        DEBUG_ERROR("No branch for (observation --> action --> observation) = ("
                    << node_info_map[root_node].observation << " --> " << action << " --> " << observation << ")");
        init(observation,state);
        return;
    }

    //------------//
    // prune tree //
    //------------//

#if 0
    /* This approach is OK but searches the whole graph. However usually only
     * few (as compared to the graph size) nodes have to be removed. */

    // set new root node and remove old one, which splits graph in two connected
    // components
    if(graph_type==FULL_DAG) {
        node_info_map[root_node].level_map_it->erase(node_info_map[root_node].observation);
    }
    erase_node(root_node);
    root_node = new_root_node;
    root_state = state;

    // find nodes that are reachable from new root node
    graph_t::NodeMap<bool> reached(graph);
    graph_util::graph_flooding(graph,reached).add_source(root_node).flood();

    // erase those that cannot be reached (but don't iterate AND erase at the
    // same time)
    vector<node_t> nodes_to_erase;
    for(node_it_t node(graph); node!=INVALID; ++node) {
        if(!reached[node]) nodes_to_erase.push_back(node);
    }
    for(node_t node : nodes_to_erase) {
        if(graph_type==FULL_DAG && type(node)==OBSERVATION_NODE) {
            node_info_map[node].level_map_it->erase(node_info_map[node].observation);
        }
        erase_node(node);
    }
#else
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
    if(graph_type==FULL_DAG) {
        node_info_map[root_node].level_map_it->erase(node_info_map[root_node].observation);
    }
    erase_node(root_node);
    root_node = new_root_node;
    root_state = state;

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
            if(graph_type==FULL_DAG && type(node)==OBSERVATION_NODE) {
                node_info_map[node].level_map_it->erase(node_info_map[node].observation);
            }
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

#endif

    // pop front of level set list
    if(graph_type==FULL_DAG) {
        level_map_list.pop_front();
    }
}

void SearchTree::toPdf(const char* file_name) const {

    graph_t::NodeMap<QString> node_map(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        node_map[node] = QString("shape=%2 label=<%3>").
            arg(node_info_map[node].type==OBSERVATION_NODE?"square":"circle").
            arg(str_rich(node));
    }

    graph_t::ArcMap<QString> arc_map(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        if(node_info_map[graph.source(arc)].type==OBSERVATION_NODE) {
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

const node_t & SearchTree::get_root_node() const {
    return root_node;
}

const SearchTree::state_handle_t & SearchTree::get_root_state() const {
    return root_state;
}

const SearchTree::graph_t & SearchTree::get_graph() const {
    return graph;
}

const SearchTree::node_info_map_t & SearchTree::get_node_info_map() const {
    return node_info_map;
}

const SearchTree::observation_handle_t SearchTree::observation(const node_t & observation_node) const {
    DEBUG_EXPECT(1,node_info_map[observation_node].type==OBSERVATION_NODE);
    if(node_info_map[observation_node].type!=OBSERVATION_NODE) {
        DEBUG_WARNING("Node " << graph.id(observation_node) << " is not a observation node");
        toPdf("tree.pdf.debug1");
    }
    return node_info_map[observation_node].observation;
}

const SearchTree::action_handle_t SearchTree::action(const node_t & action_node) const {
    DEBUG_EXPECT(1,node_info_map[action_node].type==ACTION_NODE);
    return node_info_map[action_node].action;
}

const SearchTree::NODE_TYPE SearchTree::type(const node_t & node) const {
    return node_info_map[node].type;
}

size_t SearchTree::number_of_children(const node_t & observation_node) const {
    return countOutArcs(graph, observation_node);
}

bool SearchTree::is_fully_expanded(const node_t & observation_node) const {
    DEBUG_EXPECT(1,node_info_map[observation_node].type==OBSERVATION_NODE);
    if(node_info_map[observation_node].type!=OBSERVATION_NODE) {
        DEBUG_WARNING("Node " << graph.id(observation_node) << " is not a observation node");
        toPdf("tree.pdf.debug2");
    }
    return number_of_children(observation_node)==environment->get_actions().size();
}

bool SearchTree::is_partially_expanded(const node_t & observation_node) const {
    DEBUG_EXPECT(1,node_info_map[observation_node].type==OBSERVATION_NODE);
    size_t action_node_counter = number_of_children(observation_node);
    return action_node_counter>0 && action_node_counter<environment->get_actions().size();
}

bool SearchTree::is_not_expanded(const node_t & observation_node) const {
    DEBUG_EXPECT(1,node_info_map[observation_node].type==OBSERVATION_NODE);
    return out_arc_it_t(graph,observation_node)==INVALID;
}


QString SearchTree::str(const node_t & n) const {
    bool is_observation_node = node_info_map[n].type==OBSERVATION_NODE;
    return QString("%1 (%2)").
        arg(is_observation_node?"OBSERVATION":"ACTION").
        arg(is_observation_node?
            Environment::name(*environment,node_info_map[n].observation):
            Environment::name(*environment,node_info_map[n].action));
}

QString SearchTree::str_rich(const node_t & n) const {
    bool is_observation_node = node_info_map[n].type==OBSERVATION_NODE;
    // return QString("<i>%1</i>=%2").
    //     arg(is_observation_node?"s":"a").
    //     arg(is_observation_node?node_info_map[n].observation:node_info_map[n].action);
    return QString("%1").arg(is_observation_node?Environment::name(*environment,node_info_map[n].observation):Environment::name(*environment,node_info_map[n].action));
}

double SearchTree::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}

tuple<arc_t,node_t> SearchTree::find_or_create_observation_node(const node_t & action_node,
                                                                const observation_handle_t & observation) {
    DEBUG_OUT(3,"Find observation node (action node (" << graph.id(action_node) << "): " <<
              Environment::name(*environment,node_info_map[action_node].action) << ", observation: " <<
              Environment::name(*environment,observation) << ")");

    // have node and arc both as separate variables and as tuple of references
    node_t observation_node;
    arc_t observation_arc;
    tuple<arc_t&,node_t&> arc_node_pair(observation_arc,observation_node);

    // look through children of this action node
    arc_node_pair = find_observation_node_among_children(action_node, observation);
    if(observation_node!=INVALID) {
        return arc_node_pair;
    }

    // look through all children of the action node's siblings
    if(graph_type==PARTIAL_DAG ||
       graph_type==FULL_DAG) {
        observation_node = find_observation_node_among_siblings_children(action_node, observation);
        if(observation_node!=INVALID) {
            observation_arc = graph.addArc(action_node,observation_node);
            return arc_node_pair;
        }
    }

    // look for existing node at same depth
    if(graph_type==FULL_DAG) {
        observation_node = find_observation_node_at_same_depth(action_node, observation);
        if(observation_node!=INVALID) {
            observation_arc = graph.addArc(action_node,observation_node);
            return arc_node_pair;
        }
    }

    // observation node doesn't exist --> create
    return add_observation_node(observation, action_node);
}

tuple<arc_t,node_t> SearchTree::find_or_create_action_node(const node_t & observation_node,
                                                           const action_handle_t & action) {
    for(out_arc_it_t arc(graph, observation_node); arc!=INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        if(node_info_map[action_node].action==action) {
            return make_tuple(arc, action_node);
        }
    }
    // action node doesn't exist --> create
    return add_action_node(action, observation_node);
}

std::tuple<arc_t,node_t> SearchTree::add_observation_node(observation_handle_t observation,
                                                          node_t action_node) {
    node_t observation_node = graph.addNode();
    node_info_map[observation_node].type = OBSERVATION_NODE;
    node_info_map[observation_node].observation = observation;
    node_info_map[observation_node].observation = observation;
    arc_t observation_arc = graph.addArc(action_node, observation_node);
    if(graph_type==FULL_DAG) {
        add_observation_node_to_level_map(observation_node);
    }
    DEBUG_OUT(3,"    adding observation node (" << graph.id(observation_node) << "): " <<
              Environment::name(*environment,observation));
    return make_tuple(observation_arc, observation_node);
}

std::tuple<arc_t,node_t> SearchTree::add_action_node(action_handle_t action, node_t observation_node) {
    node_t action_node = graph.addNode();
    node_info_map[action_node].type = ACTION_NODE;
    node_info_map[action_node].action = action;
    arc_t action_arc = graph.addArc(observation_node, action_node);
    DEBUG_OUT(3,"    adding action node (" << graph.id(action_node) << "): " <<
              Environment::name(*environment,action));
    return make_tuple(action_arc, action_node);
}

void SearchTree::erase_node(node_t node) {
    graph.erase(node);
}

tuple<arc_t,node_t> SearchTree::find_observation_node_among_children(const node_t & action_node,
                                                                     const observation_handle_t & observation) const {
    for(out_arc_it_t arc(graph, action_node); arc!=INVALID; ++arc) {
        node_t observation_node = graph.target(arc);
        if(node_info_map[observation_node].observation==observation) {
            return make_tuple(arc, observation_node);
        }
    }
    return tuple<arc_t,node_t>(INVALID, INVALID);
}

node_t SearchTree::find_observation_node_among_siblings_children(const node_t & action_node,
                                                                 const observation_handle_t & observation) const {
    // get common parent node
    node_t parent_node = graph.source(in_arc_it_t(graph,action_node));

    // iterate through siblings
    for(out_arc_it_t to_action_arc(graph,parent_node); to_action_arc!=INVALID; ++to_action_arc) {
        node_t sibling_node = graph.target(to_action_arc);
        // skip the given node
        if(sibling_node==action_node) continue;

        // iterate through siblings children
        for(out_arc_it_t to_observation_arc(graph, sibling_node); to_observation_arc!=lemon::INVALID; ++to_observation_arc) {
            node_t observation_node = graph.target(to_observation_arc);
            if(node_info_map[observation_node].observation==observation) {
                return observation_node;
            }
        }
    }
    return INVALID;
}

node_t SearchTree::find_observation_node_at_same_depth(const node_t & action_node,
                                                       const observation_handle_t & observation) const {
    // first get observation node at lower level
    node_t lower_level_observation_node = graph.source(in_arc_it_t(graph, action_node));
    // get level set iterator
    auto level_map_it = node_info_map[lower_level_observation_node].level_map_it;
    // go to next level
    ++level_map_it;
    // return INVALID if there are no nodes on this level
    if(level_map_it==level_map_list.end()) {
        return INVALID;
    }
    // find correct observation node or return INVALID
    auto it = level_map_it->find(observation);
    if(it!=level_map_it->end()) {
        return it->second;
    } else {
        return INVALID;
    }
}

void SearchTree::add_observation_node_to_level_map(const node_t & observation_node) {
    if(observation_node==root_node) {
        DEBUG_ERROR("Request to add root node to level set, which should be done in init() function.");
        return;
    }
    // first get observation node at lower level
    node_t action_node = graph.source(in_arc_it_t(graph, observation_node));
    node_t lower_level_observation_node = graph.source(in_arc_it_t(graph, action_node));
    auto lower_level_map_it = node_info_map[lower_level_observation_node].level_map_it;
    // go to higher level set (the one observation_node needs to be inserted in) and
    // insert observation_node or create new level with observation_node as only element
    auto higher_level_map_it = lower_level_map_it;
    ++higher_level_map_it;
    if(higher_level_map_it==level_map_list.end()) {
        level_map_list.push_back(level_map_t({make_pair(observation(observation_node),observation_node)}));
        higher_level_map_it = lower_level_map_it;
        ++higher_level_map_it;
        DEBUG_EXPECT(1,higher_level_map_it!=level_map_list.end());
    } else {
        (*higher_level_map_it)[observation(observation_node)] = observation_node;
    }
    // update node info
    node_info_map[observation_node].level_map_it = higher_level_map_it;
}
