#include "SearchTree.h"

#include <algorithm> // for std::max
#include <set>

#include <lemon/adaptors.h>     // for Undirector adapter
#include <lemon/connectivity.h> // for connected components

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
using lemon::INVALID;
using util::random_select;

SearchTree::SearchTree(const state_t & s, std::shared_ptr<Environment> env, double d):
    node_info_map(graph),
    arc_info_map(graph),
    discount(d),
    environment(env)
{
    init(s);
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

    // remove incoming arc from new root node to split graph in two connected
    // components
    {
        vector<arc_t> arcs_to_erase;
        for(in_arc_it_t arc(graph, new_root_node); arc!=INVALID; ++arc) {
            arcs_to_erase.push_back(arc);
        }
        for(auto arc : arcs_to_erase) {
            graph.erase(arc);
        }
    }

    // compute connected components (need undirected graph to use standard
    // algorithms) and erase all nodes that are not in the component of the new
    // root node
    {
        // compute components
        typedef lemon::Undirector<graph_t> ugraph_t;
        ugraph_t ugraph(graph);
        ugraph_t::NodeMap<int> component_map(ugraph);
        int component_n = lemon::connectedComponents(ugraph,component_map);
        DEBUG_EXPECT(0,component_n>1);

        // find and erase nodes
        vector<node_t> nodes_to_erase;
        int main_component = component_map[new_root_node];
        for(node_it_t node(graph); node!=INVALID; ++node) {
            if(component_map[node]!=main_component) {
                nodes_to_erase.push_back(node);
            }
        }
        for(node_t node : nodes_to_erase) {
            graph.erase(node);
        }
    }

    // set root node
    root_node = new_root_node;
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
        node_t source = graph.source(arc);
        node_t target = graph.target(arc);
        if(node_info_map[source].type==STATE_NODE) {
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
    return QString("%1").arg(is_state_node?environment->state_name(node_info_map[n].state):environment->action_name(node_info_map[n].action));
}

double SearchTree::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}
