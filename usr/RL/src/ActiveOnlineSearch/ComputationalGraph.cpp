#include "ComputationalGraph.h"

#include <lemon/maps.h>
#include <lemon/connectivity.h>

#include <cmath>
#include <set>

#include <util/graph_plotting.h>
#include <util/QtUtil.h>

#define DEBUG_LEVEL 1
#include <util/debug.h>

using namespace lemon;
using std::vector;
using std::set;
using util::Range;


ComputationalGraph::ComputationalGraph(std::shared_ptr<graph_t> g):
    graph(g),
    node_labels(*graph),
    node_values(*graph),
    node_differentials(*graph),
    node_variables(*graph),
    arc_values(*graph),
    node_functions(*graph),
    arc_functions(*graph) {}

ComputationalGraph::ComputationalGraph(): ComputationalGraph(std::make_shared<graph_t>()) {}

void ComputationalGraph::assign_values(std::vector<double> values, TYPE a) {
    switch(a) {
    case VALUES:
        if(values.size()!=input_nodes.size()) {
            DEBUG_WARNING("Number of values to assign does not match number of input nodes.");
        }
        {
            auto value_it = values.begin();
            auto node_it = input_nodes.begin();
            while(value_it!=values.end() && node_it!=input_nodes.end()) {
                node_values[*node_it] = *value_it;
                ++value_it;
                ++node_it;
            }
        }
        break;
    case FORWARD:
        if(values.size()!=input_nodes.size()) {
            DEBUG_WARNING("Number of values to assign does not match number of input nodes.");
        }
        {
            auto value_it = values.begin();
            auto node_it = input_nodes.begin();
            while(value_it!=values.end() && node_it!=input_nodes.end()) {
                node_differentials[*node_it] = *value_it;
                ++value_it;
                ++node_it;
            }
        }
        break;
    case REVERSE:
        if(values.size()!=output_nodes.size()) {
            DEBUG_WARNING("Number of values to assign does not match number of output nodes.");
        }
        {
            auto value_it = values.begin();
            auto node_it = output_nodes.begin();
            while(value_it!=values.end() && node_it!=output_nodes.end()) {
                node_differentials[*node_it] = *value_it;
                ++value_it;
                ++node_it;
            }
        }
        break;
    default:
        DEBUG_DEAD_LINE;
    }
}

void ComputationalGraph::propagate_values(TYPE p) {
    // status of the nodes
    graph_t::NodeMap<bool> done(*graph,false);
    set<node_t> active_nodes;
    set<node_t> pending_nodes;

    switch(p) {
    case VALUES:
    case FORWARD:
        // kick off propagation from input nodes
        DEBUG_OUT(2, "Processing input nodes...");
        for(node_t node : input_nodes) {
            done[node] = true;
            DEBUG_OUT(3, "        Input node '" << node_labels[node] << "'");
            for(out_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
                node_t target_node = graph->target(arc);
                active_nodes.insert(target_node);
                DEBUG_OUT(3, "            add '" << node_labels[target_node] << "' to active set");
            }
        }
        break;
    case REVERSE:
        // kick off propagation from output nodes
        DEBUG_OUT(2, "Processing output nodes...");
        for(node_t node : output_nodes) {
            done[node] = true;
            DEBUG_OUT(3, "        Output node '" << node_labels[node] << "'");
            for(in_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
                node_t source_node = graph->source(arc);
                active_nodes.insert(source_node);
                DEBUG_OUT(3, "            add '" << node_labels[source_node] << "' to active set");
            }
        }
        break;
    default:
        DEBUG_DEAD_LINE;
    }

    // keep processing active nodes till end
    DEBUG_OUT(2, "    Looping through active nodes...");
    while(active_nodes.size()>0) {
        DEBUG_OUT(2, "    New iteration...");
        set<node_t> new_active_nodes;
        set<node_t> new_done;
        set<node_t> move_to_pending;
        for(node_t node : active_nodes) {
            DEBUG_OUT(2, "            Checking active node '" << node_labels[node] << "'");
            bool all_inputs_available = true;
            switch(p) {
            case VALUES:
            case FORWARD:
                for(in_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
                    if(!done[graph->source(arc)]) {
                        all_inputs_available = false;
                        break;
                    }
                }
                break;
            case REVERSE:
                for(out_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
                    if(!done[graph->target(arc)]) {
                        all_inputs_available = false;
                        break;
                    }
                }
                break;
            default:
                DEBUG_DEAD_LINE;
            }
            if(all_inputs_available) {
                done[node] = true;
                new_done.insert(node);
                DEBUG_OUT(3, "                All inputs available --> compute value");
                evaluate_node(node, p);
                switch(p) {
                case VALUES:
                case FORWARD:
                    for(out_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
                        node_t target_node = graph->target(arc);
                        new_active_nodes.insert(target_node);
                        DEBUG_OUT(4, "                    add '" << node_labels[target_node] << "' to active set");
                    }
                    break;
                case REVERSE:
                    for(in_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
                        node_t source_node = graph->source(arc);
                        new_active_nodes.insert(source_node);
                        DEBUG_OUT(4, "                    add '" << node_labels[source_node] << "' to active set");
                    }
                    break;
                default:
                    DEBUG_DEAD_LINE;
                }
            } else {
                DEBUG_OUT(3, "                Some input pending --> move node to pending set");
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

void ComputationalGraph::compute_values(std::vector<double> values) {
    assign_values(values, VALUES);
    propagate_values(VALUES);
}

void ComputationalGraph::forward_accumulation(std::vector<double> values,
                                               std::vector<double> differentials) {
    compute_values(values);
    assign_values(differentials, FORWARD);
    propagate_values(FORWARD);
}

void ComputationalGraph::reverse_accumulation(std::vector<double> values,
                                               std::vector<double> differentials) {
    compute_values(values);
    assign_values(differentials, REVERSE);
    propagate_values(REVERSE);
}

bool ComputationalGraph::check_derivatives(std::vector<double> values,
                                            double delta,
                                            double epsilon_absolute,
                                            double epsilon_relative) {
    // compute all values
    compute_values(values);
    // make a copy of the derivatives
    graph_t::ArcMap<double> arc_values_copy(*graph);
    mapCopy(*graph, arc_values, arc_values_copy);
    // change the value of all nodes by delta and nummerically check derivatives
    bool ok = true;
    for(node_it_t node(*graph); node!=INVALID; ++node) {
        DEBUG_OUT(2,"Checking derivatives w.r.t " <<
                  node_labels[node] << " (id" << graph->id(node) << ")");
        // get dependent nodes (the ones we need to check)
        vector<node_t> dependent_nodes;
        vector<arc_t> dependent_arcs;
        for(out_arc_it_t arc(*graph, node); arc!=INVALID; ++arc) {
            dependent_nodes.push_back(graph->target(arc));
            dependent_arcs.push_back(arc);
        }
        // compute finite differences
        double old_value = node_values[node];
        vector<double> values_1;
        vector<double> values_2;
        {
            // +delta/2
            node_values[node] = old_value+delta/2;
            for(node_t dep_node : dependent_nodes) {
                values_1.push_back(evaluate_node(dep_node, VALUES));
            }
            // -delta/2
            node_values[node] = old_value-delta/2;
            for(node_t dep_node : dependent_nodes) {
                values_2.push_back(evaluate_node(dep_node, VALUES));
            }
            // reset
            node_values[node] = old_value;
        }
        // check against derivatives
        int num_vars = dependent_nodes.size();
        DEBUG_EXPECT(1, dependent_nodes.size()==num_vars &&
                     dependent_arcs.size()==num_vars &&
                     values_1.size()==num_vars &&
                     values_2.size()==num_vars);
        for(int idx : Range(num_vars)) {
            DEBUG_OUT(2,"Checking derivative of " << node_labels[dependent_nodes[idx]] <<
                      " (id" << graph->id(dependent_nodes[idx]) << ") w.r.t " <<
                      node_labels[node] << " (id" << graph->id(node) << ")");
            double numerical_derivative = (values_1[idx] - values_2[idx])/delta;
            double analytical_derivative = arc_values_copy[dependent_arcs[idx]];
            if(fabs(numerical_derivative-analytical_derivative)>epsilon_absolute and
               fabs(numerical_derivative/analytical_derivative-1)>epsilon_relative) {
                ok = false;
                DEBUG_WARNING("Partial derivative of " << node_labels[dependent_nodes[idx]] <<
                              " (id" << graph->id(dependent_nodes[idx]) << ") w.r.t " <<
                              node_labels[node] <<
                              " (id" << graph->id(node) << ") is out of bounds (numerical=" <<
                              numerical_derivative << " / analytical=" <<
                              analytical_derivative << ")");
            }
        }
    }
    return ok;
}

void ComputationalGraph::plot_graph(const char* file_name) const {
    graph_t::NodeMap<QString> node_properties(*graph);
    for(node_it_t node(*graph); node!=INVALID; ++node) {
        node_properties[node] = QString("label=<%1=%2<BR/><I>d</I>%1=%3>").
            arg(node_labels[node]).
            arg(node_values[node]).
            arg(node_differentials[node]);
    }
    graph_t::ArcMap<QString> arc_properties(*graph);
    for(arc_it_t arc(*graph); arc!=INVALID; ++arc) {
        arc_properties[arc] = QString("label=<%1>").
            arg(arc_values[arc]);
    }
    util::graph_to_pdf(file_name, *graph, "", &node_properties, "", &arc_properties);
}

double ComputationalGraph::evaluate_node(const node_t & node, TYPE e) {
    switch(e) {
    case VALUES:
    {
        // get variables
        vector<QString> variables = node_variables[node];
        vector<double> values;
        for(QString var : variables) {
            bool found = false;
            for(in_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
                node_t source_node = graph->source(arc);
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
            for(in_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
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
        for(in_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
            double val = arc_functions[arc](values);
            arc_values[arc] = val;
        }

        // evaluate node
        double val = node_functions[node](values);
        node_values[node] = val;
        return val;
    }
    case FORWARD:
    {
        // evaluate arcs
        double val = 0;
        for(in_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
            val += node_differentials[graph->source(arc)]*arc_values[arc];
        }
        node_differentials[node] = val;
        return val;
    }
    case REVERSE:
    {
        // evaluate arcs
        double val = 0;
        for(out_arc_it_t arc(*graph,node); arc!=INVALID; ++arc) {
            val += node_differentials[graph->target(arc)]*arc_values[arc];
        }
        node_differentials[node] = val;
        return val;
    }
    default:
        DEBUG_DEAD_LINE;
        return 0;
    }
}

ComputationalGraph::node_t ComputationalGraph::add_node(QString label,
                                                          std::vector<QString> variables,
                                                          function_t function) {
    node_t node = graph->addNode();
    node_labels[node] = label;
    node_values[node] = NAN;
    node_differentials[node] = NAN;
    node_variables[node] = variables;
    node_functions[node] = function;
    return node;
}

ComputationalGraph::arc_t ComputationalGraph::add_arc(node_t from,
                                                        node_t to,
                                                        function_t function) {
    arc_t arc = graph->addArc(from, to);
    arc_values[arc] = NAN;
    arc_functions[arc] = function;
    return arc;
}

vector<double> ComputationalGraph::get_input_differentials() const {
    vector<double> values;
    for(node_t node : input_nodes) {
        values.push_back(node_differentials[node]);
    }
    return values;
}

vector<double> ComputationalGraph::get_output_differentials() const {
    vector<double> values;
    for(node_t node : output_nodes) {
        values.push_back(node_differentials[node]);
    }
    return values;
}

vector<double> ComputationalGraph::get_output_values() const {
    vector<double> values;
    for(node_t node : output_nodes) {
        values.push_back(node_values[node]);
    }
    return values;
}

void ComputationalGraph::set_input_nodes(std::vector<node_t> n) {
    input_nodes = n;
}

void ComputationalGraph::set_output_nodes(std::vector<node_t> n) {
    output_nodes = n;
}

bool ComputationalGraph::check_graph_structure(bool reset_input_nodes,
                                                bool reset_output_nodes) {
    bool ok = true;

    // check for cycles
    if(!dag(*graph)) {
        DEBUG_WARNING("Graph contains cycles");
        ok = false;
    }

    // check input/output nodes
    set<node_t> new_input_nodes, new_output_nodes;
    for(node_it_t node(*graph); node!=INVALID; ++node) {
        if(in_arc_it_t(*graph,node)==INVALID) new_input_nodes.insert(node);
        if(out_arc_it_t(*graph,node)==INVALID) new_output_nodes.insert(node);
    }
    // input
    if(reset_input_nodes) input_nodes.assign(new_input_nodes.begin(), new_input_nodes.end());
    set<node_t> old_input_nodes(input_nodes.begin(), input_nodes.end());
    if(old_input_nodes!=new_input_nodes) {
        DEBUG_WARNING("Provided and actual input nodes do not match");
        ok = false;
    }
    // output
    if(reset_output_nodes) output_nodes.assign(new_output_nodes.begin(), new_output_nodes.end());
    set<node_t> old_output_nodes(output_nodes.begin(), output_nodes.end());
    if(old_output_nodes!=new_output_nodes) {
        DEBUG_WARNING("Provided and actual output nodes do not match");
        ok = false;
    }

    return ok;
}
