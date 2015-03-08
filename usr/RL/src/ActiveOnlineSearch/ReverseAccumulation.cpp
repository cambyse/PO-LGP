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
    node_differentials(graph),
    node_variables(graph),
    arc_values(graph),
    node_functions(graph),
    arc_functions(graph) {

    // create graph
    node_t alpha = add_node("alpha");
    node_t t = add_node("t");
    node_t w = add_node("w");

    node_t v1 = add_node("v1", {"t"}, [](vector<double> v)->double{return sqrt(v[0]);});
    node_t v2 = add_node("v2", {"alpha","v1"}, [](vector<double> v)->double{return -v[0]*v[1];});
    node_t v3 = add_node("v3", {"v2"}, [](vector<double> v)->double{return exp(v[0]);});
    node_t v4 = add_node("v4", {"w","v3","v1"}, [](vector<double> v)->double{return v[0]*v[1]*v[2];});

    node_t x = add_node("x", {"v4"}, [](vector<double> v)->double{return sin(v[0]);});
    node_t y = add_node("y", {"v4"}, [](vector<double> v)->double{return cos(v[0]);});

    add_arc(alpha, v2, [](vector<double> v)->double{return -v[1];});
    add_arc(t, v1, [](vector<double> v)->double{return -1/(2*sqrt(v[0]));});
    add_arc(w, v4, [](vector<double> v)->double{return v[1]*v[2];});
    add_arc(v1, v2, [](vector<double> v)->double{return -v[0];});
    add_arc(v1, v4, [](vector<double> v)->double{return v[0]*v[1];});
    add_arc(v2, v3, [](vector<double> v)->double{return exp(v[0]);});
    add_arc(v3, v4, [](vector<double> v)->double{return v[0]*v[1];});
    add_arc(v4, x, [](vector<double> v)->double{return cos(v[0]);});
    add_arc(v4, y, [](vector<double> v)->double{return -sin(v[0]);});

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
                evaluate_node(node);
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
        node_properties[node] = QString("label=<%1=%2<BR/><I>d</I>%1=%3>").
            arg(node_labels[node]).
            arg(node_values[node]).
            arg(node_differentials[node]);
    }
    graph_t::ArcMap<QString> arc_properties(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        arc_properties[arc] = QString("label=<%1>").
            arg(arc_values[arc]);
    }
    util::graph_to_pdf(file_name, graph, "", &node_properties, "", &arc_properties);
}

double ReverseAccumulation::evaluate_node(const node_t & node) {
    // get variables
    vector<QString> variables = node_variables[node];
    vector<double> values;
    for(QString var : variables) {
        bool found = false;
        for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
            node_t source_node = graph.source(arc);
            if(node_labels[source_node]==var) {
                found = true;
                values.push_back(node_values[source_node]);
            }
        }
        if(!found) {
            DEBUG_ERROR("Could not find node with label '" << var << "' while computing value of node '" << node_labels[node] << "'");
            values.push_back(NAN);
        }
    }

    // debug output
    IF_DEBUG(1) {
        int input_size = 0;
        for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
            ++input_size;
        }
        if(input_size!=variables.size()) {
            DEBUG_WARNING("Number of input values ("
                          << input_size
                          << ") does not match number of variables given ("
                          << variables.size() << ")");
        }
    }

    // evaluate arcs
    for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
        double val = arc_functions[arc](values);
        arc_values[arc] = val;
    }

    // evaluate node
    double val = node_functions[node](values);
    node_values[node] = val;
    return val;
}

ReverseAccumulation::node_t ReverseAccumulation::add_node(QString label,
                                                          std::vector<QString> variables,
                                                          function_t function) {
    node_t node = graph.addNode();
    node_labels[node] = label;
    node_values[node] = NAN;
    node_differentials[node] = NAN;
    node_variables[node] = variables;
    node_functions[node] = function;
    return node;
}

ReverseAccumulation::arc_t ReverseAccumulation::add_arc(node_t from,
                                                        node_t to,
                                                        function_t function) {
    arc_t arc = graph.addArc(from, to);
    arc_values[arc] = NAN;
    arc_functions[arc] = function;
    return arc;
}
