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

void SearchTree::init(const state_handle_t & s) {
    graph.clear();
    root_node = graph.addNode();
    node_info_map[root_node].state=s;
    if(graph_type==FULL_DAG) {
        level_map_list.assign({level_map_t({make_pair(s,root_node)})});
        node_info_map[root_node].level_map_it = level_map_list.begin();
    }
}

void SearchTree::prune(const action_handle_t & action, const state_handle_t & state) {
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
        init(state);
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
        node_info_map[root_node].level_map_it->erase(node_info_map[root_node].state);
    }
    erase_node(root_node);
    root_node = new_root_node;

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
        if(graph_type==FULL_DAG && type(node)==STATE_NODE) {
            node_info_map[node].level_map_it->erase(node_info_map[node].state);
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
        node_info_map[root_node].level_map_it->erase(node_info_map[root_node].state);
    }
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
            if(graph_type==FULL_DAG && type(node)==STATE_NODE) {
                node_info_map[node].level_map_it->erase(node_info_map[node].state);
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

const node_t & SearchTree::get_root_node() const {
    return root_node;
}

const SearchTree::graph_t & SearchTree::get_graph() const {
    return graph;
}

const SearchTree::node_info_map_t & SearchTree::get_node_info_map() const {
    return node_info_map;
}

const SearchTree::state_handle_t SearchTree::state(const node_t & state_node) const {
    DEBUG_EXPECT(1,node_info_map[state_node].type==STATE_NODE);
    if(node_info_map[state_node].type!=STATE_NODE) {
        DEBUG_WARNING("Node " << graph.id(state_node) << " is not a state node");
        toPdf("tree.pdf.debug1");
    }
    return node_info_map[state_node].state;
}

const SearchTree::action_handle_t SearchTree::action(const node_t & action_node) const {
    DEBUG_EXPECT(1,node_info_map[action_node].type==ACTION_NODE);
    return node_info_map[action_node].action;
}

const SearchTree::NODE_TYPE SearchTree::type(const node_t & node) const {
    return node_info_map[node].type;
}

size_t SearchTree::number_of_children(const node_t & state_node) const {
    return countOutArcs(graph, state_node);
}

bool SearchTree::is_fully_expanded(const node_t & state_node) const {
    DEBUG_EXPECT(1,node_info_map[state_node].type==STATE_NODE);
    if(node_info_map[state_node].type!=STATE_NODE) {
        DEBUG_WARNING("Node " << graph.id(state_node) << " is not a state node");
        toPdf("tree.pdf.debug2");
    }
    return number_of_children(state_node)==environment->get_actions().size();
}

bool SearchTree::is_partially_expanded(const node_t & state_node) const {
    DEBUG_EXPECT(1,node_info_map[state_node].type==STATE_NODE);
    size_t action_node_counter = number_of_children(state_node);
    return action_node_counter>0 && action_node_counter<environment->get_actions().size();
}

bool SearchTree::is_not_expanded(const node_t & state_node) const {
    DEBUG_EXPECT(1,node_info_map[state_node].type==STATE_NODE);
    return out_arc_it_t(graph,state_node)==INVALID;
}


QString SearchTree::str(const node_t & n) const {
    bool is_state_node = node_info_map[n].type==STATE_NODE;
    return QString("%1 (%2)").
        arg(is_state_node?"STATE":"ACTION").
        arg(is_state_node?
            Environment::name(*environment,node_info_map[n].state):
            Environment::name(*environment,node_info_map[n].action));
}

QString SearchTree::str_rich(const node_t & n) const {
    bool is_state_node = node_info_map[n].type==STATE_NODE;
    // return QString("<i>%1</i>=%2").
    //     arg(is_state_node?"s":"a").
    //     arg(is_state_node?node_info_map[n].state:node_info_map[n].action);
    return QString("%1").arg(is_state_node?Environment::name(*environment,node_info_map[n].state):Environment::name(*environment,node_info_map[n].action));
}

double SearchTree::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}

tuple<arc_t,node_t> SearchTree::find_or_create_state_node(const node_t & action_node,
                                                          const state_handle_t & state) {
    DEBUG_OUT(3,"Find state node (action node (" << graph.id(action_node) << "): " <<
              Environment::name(*environment,node_info_map[action_node].action) << ", state: " <<
              Environment::name(*environment,state) << ")");

    // have node and arc both as separate variables and as tuple of references
    node_t state_node;
    arc_t state_arc;
    tuple<arc_t&,node_t&> arc_node_pair(state_arc,state_node);

    // look through children of this action node
    arc_node_pair = find_state_node_among_children(action_node, state);
    if(state_node!=INVALID) {
        return arc_node_pair;
    }

    // look through all children of the action node's siblings
    if(graph_type==PARTIAL_DAG ||
       graph_type==FULL_DAG) {
        state_node = find_state_node_among_siblings_children(action_node, state);
        if(state_node!=INVALID) {
            state_arc = graph.addArc(action_node,state_node);
            return arc_node_pair;
        }
    }

    // look for existing node at same depth
    if(graph_type==FULL_DAG) {
        state_node = find_state_node_at_same_depth(action_node, state);
        if(state_node!=INVALID) {
            state_arc = graph.addArc(action_node,state_node);
            return arc_node_pair;
        }
    }

    // state node doesn't exist --> create
    return add_state_node(state, action_node);
}

tuple<arc_t,node_t> SearchTree::find_or_create_action_node(const node_t & state_node,
                                                           const action_handle_t & action) {
    for(out_arc_it_t arc(graph, state_node); arc!=INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        if(node_info_map[action_node].action==action) {
            return make_tuple(arc, action_node);
        }
    }
    // action node doesn't exist --> create
    return add_action_node(action, state_node);
}

std::tuple<arc_t,node_t> SearchTree::add_state_node(state_handle_t state, node_t action_node) {
    node_t state_node = graph.addNode();
    node_info_map[state_node].type = STATE_NODE;
    node_info_map[state_node].state = state;
    arc_t state_arc = graph.addArc(action_node, state_node);
    if(graph_type==FULL_DAG) {
        add_state_node_to_level_map(state_node);
    }
    DEBUG_OUT(3,"    adding state node (" << graph.id(state_node) << "): " <<
              Environment::name(*environment,state));
    return make_tuple(state_arc, state_node);
}

std::tuple<arc_t,node_t> SearchTree::add_action_node(action_handle_t action, node_t state_node) {
    node_t action_node = graph.addNode();
    node_info_map[action_node].type = ACTION_NODE;
    node_info_map[action_node].action = action;
    arc_t action_arc = graph.addArc(state_node, action_node);
    DEBUG_OUT(3,"    adding action node (" << graph.id(action_node) << "): " <<
              Environment::name(*environment,action));
    return make_tuple(action_arc, action_node);
}

void SearchTree::erase_node(node_t node) {
    graph.erase(node);
}

tuple<arc_t,node_t> SearchTree::find_state_node_among_children(const node_t & action_node,
                                                               const state_handle_t & state) const {
    for(out_arc_it_t arc(graph, action_node); arc!=INVALID; ++arc) {
        node_t state_node = graph.target(arc);
        if(node_info_map[state_node].state==state) {
            return make_tuple(arc, state_node);
        }
    }
    return tuple<arc_t,node_t>(INVALID, INVALID);
}

node_t SearchTree::find_state_node_among_siblings_children(const node_t & action_node,
                                                           const state_handle_t & state) const {
    // get common parent node
    node_t parent_node = graph.source(in_arc_it_t(graph,action_node));

    // iterate through siblings
    for(out_arc_it_t to_action_arc(graph,parent_node); to_action_arc!=INVALID; ++to_action_arc) {
        node_t sibling_node = graph.target(to_action_arc);
        // skip the given node
        if(sibling_node==action_node) continue;

        // iterate through siblings children
        for(out_arc_it_t to_state_arc(graph, sibling_node); to_state_arc!=lemon::INVALID; ++to_state_arc) {
            node_t state_node = graph.target(to_state_arc);
            if(node_info_map[state_node].state==state) {
                return state_node;
            }
        }
    }
    return INVALID;
}

node_t SearchTree::find_state_node_at_same_depth(const node_t & action_node,
                                                 const state_handle_t & state) const {
    // first get state node at lower level
    node_t lower_level_state_node = graph.source(in_arc_it_t(graph, action_node));
    // get level set iterator
    auto level_map_it = node_info_map[lower_level_state_node].level_map_it;
    // go to next level
    ++level_map_it;
    // return INVALID if there are no nodes on this level
    if(level_map_it==level_map_list.end()) {
        return INVALID;
    }
    // find correct state node or return INVALID
    auto it = level_map_it->find(state);
    if(it!=level_map_it->end()) {
        return it->second;
    } else {
        return INVALID;
    }
}

void SearchTree::add_state_node_to_level_map(const node_t & state_node) {
    if(state_node==root_node) {
        DEBUG_ERROR("Request to add root node to level set, which should be done in init() function. Calling init() instead.");
        init(state(state_node));
    }
    // first get state node at lower level
    node_t action_node = graph.source(in_arc_it_t(graph, state_node));
    node_t lower_level_state_node = graph.source(in_arc_it_t(graph, action_node));
    auto lower_level_map_it = node_info_map[lower_level_state_node].level_map_it;
    // go to higher level set (the one state_node needs to be inserted in) and
    // insert state_node or create new level with state_node as only element
    auto higher_level_map_it = lower_level_map_it;
    ++higher_level_map_it;
    if(higher_level_map_it==level_map_list.end()) {
        level_map_list.push_back(level_map_t({make_pair(state(state_node),state_node)}));
        higher_level_map_it = lower_level_map_it;
        ++higher_level_map_it;
        DEBUG_EXPECT(1,higher_level_map_it!=level_map_list.end());
    } else {
        (*higher_level_map_it)[state(state_node)] = state_node;
    }
    // update node info
    node_info_map[state_node].level_map_it = higher_level_map_it;
}
