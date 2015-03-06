#include "ReverseAccumulation.h"

#include <cmath>
#include <set>

#include <util/graph_plotting.h>
#include <util/QtUtil.h>

#define DEBUG_LEVEL 4
#include <util/debug.h>

using namespace lemon;
using std::vector;
using std::set;

ReverseAccumulation::ReverseAccumulation():
    node_labels(graph),
    node_values(graph),
    node_functions(graph) {

    // create graph
    auto alpha = graph.addNode();
    node_labels[alpha] = "alpha";
    node_functions[alpha] = {{},[](vector<double>)->double{return NAN;}};
    auto t = graph.addNode();
    node_labels[t] = "t";
    node_functions[t] = {{},[](vector<double>)->double{return NAN;}};
    auto w = graph.addNode();
    node_labels[w] = "w";
    node_functions[w] = {{},[](vector<double>)->double{return NAN;}};

    auto v1 = graph.addNode();
    node_labels[v1] = "v1";
    node_functions[v1] = {{"t"},[](vector<double> v)->double{return sqrt(v[0]);}};
    auto v2 = graph.addNode();
    node_labels[v2] = "v2";
    node_functions[v2] = {{"alpha","v1"},[](vector<double> v)->double{return -v[0]*v[1];}};
    auto v3 = graph.addNode();
    node_labels[v3] = "v3";
    node_functions[v3] = {{"v2"},[](vector<double> v)->double{return exp(v[0]);}};
    auto v4 = graph.addNode();
    node_labels[v4] = "v4";
    node_functions[v4] = {{"w","v3","v1"},[](vector<double> v)->double{return v[0]*v[1]*v[2];}};

    auto x = graph.addNode();
    node_labels[x] = "x";
    node_functions[x] = {{"v4"},[](vector<double> v)->double{return sin(v[0]);}};
    auto y = graph.addNode();
    node_labels[y] = "y";
    node_functions[y] = {{"v4"},[](vector<double> v)->double{return cos(v[0]);}};

    graph.addArc(alpha,v2);
    graph.addArc(t,v1);
    graph.addArc(w,v4);
    graph.addArc(v1,v2);
    graph.addArc(v1,v4);
    graph.addArc(v2,v3);
    graph.addArc(v3,v4);
    graph.addArc(v4,x);
    graph.addArc(v4,y);

    // initialize values
    for(node_it_t node(graph); node!=INVALID; ++node) {
        node_values[node] = NAN;
    }

    // fill lists of input/output nodes
    input_nodes.push_back(alpha);
    input_nodes.push_back(w);
    input_nodes.push_back(t);

    output_nodes.push_back(x);
    output_nodes.push_back(y);
}

void ReverseAccumulation::assign_values(std::vector<double> values) {
    auto value_it = values.begin();
    auto node_it = input_nodes.begin();
    while(value_it!=values.end() && node_it!=input_nodes.end()) {
        node_values[*node_it] = *value_it;
        ++value_it;
        ++node_it;
    }
}

void ReverseAccumulation::propagate_values() {
    graph_t::NodeMap<bool> done(graph,false);
    set<node_t> active_nodes;
    set<node_t> pending_nodes;
    DEBUG_OUT(2, "Processing input nodes...");
    for(node_t node : input_nodes) {
        done[node] = true;
        DEBUG_OUT(3, "    Input node '" << node_labels[node] << "'");
        for(out_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
            node_t target_node = graph.target(arc);
            active_nodes.insert(target_node);
            DEBUG_OUT(3, "        add '" << node_labels[target_node] << "' to active set");
        }
    }
    DEBUG_OUT(2, "Looping through active nodes...");
    while(active_nodes.size()>0) {
        DEBUG_OUT(2, "New iteration...");
        set<node_t> new_active_nodes;
        set<node_t> new_done;
        set<node_t> move_to_pending;
        for(node_t node : active_nodes) {
            DEBUG_OUT(2, "        Checking active node '" << node_labels[node] << "'");
            bool all_inputs_available = true;
            for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
                if(!done[graph.source(arc)]) {
                    all_inputs_available = false;
                    break;
                }
            }
            if(all_inputs_available) {
                DEBUG_OUT(3, "            All inputs available --> compute value");
                double value = map_function(node, node_functions[node]);
                node_values[node] = value;
                done[node] = true;
                new_done.insert(node);
                for(out_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
                    node_t target_node = graph.target(arc);
                    new_active_nodes.insert(target_node);
                    DEBUG_OUT(4, "                add '" << node_labels[target_node] << "' to active set");
                }
            } else {
                DEBUG_OUT(3, "            Some input pending --> move node to pending set");
                move_to_pending.insert(node);
            }
        }
        //-------------//
        // update sets //
        //-------------//
        // swap old and new active
        active_nodes.swap(new_active_nodes);
        // remove done nodes from active and pending (possibly includes new
        // active nodes that could be done in the same iteration)
        DEBUG_OUT(4,"    Done nodes:");
        for(node_t node : new_done) {
            DEBUG_OUT(4,"        " << node_labels[node]);
            active_nodes.erase(node);
            pending_nodes.erase(node);
        }
        // print new active nodes
        IF_DEBUG(4) {
            DEBUG_OUT(4,"    New active nodes:");
            for(node_t node : active_nodes) {
                DEBUG_OUT(4,"        " << node_labels[node]);
            }
        }
        DEBUG_OUT(4,"    Move to pending:");
        for(node_t node : move_to_pending) {
            DEBUG_OUT(4,"        " << node_labels[node]);
            pending_nodes.insert(node);
        }
    }
    if(pending_nodes.size()>0 || active_nodes.size()>0) {
        DEBUG_ERROR("Propagating values did not complete");
    }
}

void ReverseAccumulation::plot_graph(const char* file_name) const {
    graph_t::NodeMap<QString> node_properties(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        node_properties[node] = QString("label=<%1=%2>").
            arg(node_labels[node]).
            arg(node_values[node]);
    }
    util::graph_to_pdf(file_name, graph, "", &node_properties);
}

double ReverseAccumulation::map_function(const node_t & node,
                                         const std::pair<std::vector<QString>, node_function_t> & func) const {
    vector<double> values;
    for(QString label : func.first) {
        bool found = false;
        for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
            node_t source_node = graph.source(arc);
            if(node_labels[source_node]==label) {
                found = true;
                values.push_back(node_values[source_node]);
            }
        }
        if(!found) {
            DEBUG_ERROR("Could not find node with label '" << label << "' while computing value of node '" << node_labels[node] << "'");
            values.push_back(NAN);
        }
    }

    IF_DEBUG(1) {
        int input_size = 0;
        for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
            ++input_size;
        }
        if(input_size!=func.first.size()) {
            DEBUG_WARNING("Number of input values ("
                          << input_size
                          << ") does not match number of variables labels given ("
                          << func.first.size() << ")");
        }
    }

    return func.second(values);
}
