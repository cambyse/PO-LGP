#include "ComputationalConstGraph.h"

#include <lemon/maps.h>
#include <lemon/connectivity.h>
#include <lemon/adaptors.h>
#include <lemon/concepts/digraph.h>

#include <cmath>
#include <set>
#include <algorithm>

#include <util/graph_plotting.h>
#include <util/QtUtil.h>

#include "graph_util.h"

#define DEBUG_LEVEL 0
#include <util/debug.h>

using namespace lemon;
using namespace graph_util;
using std::vector;
using std::set;
using util::Range;

typedef ComputationalConstGraph CG;

CG::ComputationalConstGraph(): ComputationalConstGraph(dummy_graph)
{}

CG::ComputationalConstGraph(const graph_t & graph):
    graph(graph),
    node_labels(graph),
    node_values(graph),
    node_differentials(graph),
    arc_values(graph),
    node_variables(graph),
    node_functions(graph),
    arc_functions(graph)
{}

void CG::assign_values(std::vector<double> values, std::vector<node_t> nodes) {
    if(values.size()!=nodes.size()) {
        DEBUG_WARNING("Number of values to assign does not match number of nodes.");
    }
    for(int idx : Range(std::min(values.size(),nodes.size()))) {
        node_values[nodes[idx]] = values[idx];
    }
}

void CG::assign_differentials(std::vector<double> values, std::vector<node_t> nodes) {
    if(values.size()!=nodes.size()) {
        DEBUG_WARNING("Number of values to assign does not match number of nodes.");
    }
    for(int idx : Range(std::min(values.size(),nodes.size()))) {
        node_differentials[nodes[idx]] = values[idx];
    }
}

// void CG::propagate_values(TYPE p, std::vector<node_t> changed_nodes) {

//     switch(p) {
//     case VALUES:
//         DEBUG_OUT(1, "Propagating values...");
//         break;
//     case FORWARD:
//         DEBUG_OUT(1, "Propagating differentials (forward)...");
//         break;
//     case REVERSE:
//         DEBUG_OUT(1, "Propagating differentials (reverse)...");
//         break;
//     default:
//         DEBUG_DEAD_LINE;
//     }

//     // status of the nodes
//     set<node_t> active_nodes;
//     set<node_t> pending_nodes;

//     // find nodes that have to be processed
//     graph_t::NodeMap<bool> to_be_processed(graph,true);
//     switch(p) {
//     case VALUES:
//     case FORWARD: {
//         // simple depth first search from changed nodes
//         Dfs<graph_t> search(graph);
//         search.reachedMap(to_be_processed).init();
//         for_each(changed_nodes.begin(), changed_nodes.end(), [&](node_t n){search.addSource(n);});
//         search.start();
//         break;
//     }
//     case REVERSE: {
//         // depth first search in reverse graph from changed nodes TODO: Somehow
//         // directly use to_be_processed map as ReachedMap as above (type does
//         // not match) to avoid the for loop for copying the values.
//         typedef ReverseDigraph<const graph_t> rev_graph_t;
//         Dfs<rev_graph_t> search(reverseDigraph(graph));
//         search.init();
//         for_each(changed_nodes.begin(), changed_nodes.end(), [&](node_t n){search.addSource(n);});
//         search.start();
//         for(node_it_t node(graph); node!=INVALID; ++node) {
//             to_be_processed[node] = search.reached(node);
//         }
//         break;
//     }
//     default:
//         DEBUG_DEAD_LINE;
//     }

//     // kick off propagation from changed nodes and remove changed nodes from
//     // to-be-processed
//     DEBUG_OUT(2, "    Kick-off from changed nodes...");
//     for(node_t node : changed_nodes) {
//         to_be_processed[node] = false;
//         DEBUG_OUT(3, "        '" << node_labels[node] << "'");
//         switch(p) {
//         case VALUES:
//         case FORWARD:
//             for(out_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
//                 node_t target_node = graph.target(arc);
//                 active_nodes.insert(target_node);
//                 DEBUG_OUT(3, "            add '" << node_labels[target_node] << "' to active set");
//             }
//             break;
//         case REVERSE:
//             for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
//                 node_t source_node = graph.source(arc);
//                 active_nodes.insert(source_node);
//                 DEBUG_OUT(3, "            add '" << node_labels[source_node] << "' to active set");
//             }
//             break;
//         default:
//             DEBUG_DEAD_LINE;
//         }
//     }

//     // print nodes to be processed
//     IF_DEBUG(3) {
//         DEBUG_OUT(3,"    Nodes to be processed:");
//         for(node_it_t node(graph); node!=INVALID; ++node) {
//             if(to_be_processed[node]) {
//                 DEBUG_OUT(3,"        '" << node_labels[node] << "'");
//             }
//         }
//     }

//     // keep processing active nodes till end
//     DEBUG_OUT(2, "    Looping through active nodes...");
//     while(active_nodes.size()>0) {
//         DEBUG_OUT(2, "    New iteration...");
//         set<node_t> new_active_nodes;
//         set<node_t> new_processed;
//         set<node_t> move_to_pending;
//         for(node_t node : active_nodes) {
//             DEBUG_OUT(2, "            Checking active node '" << node_labels[node] << "'");
//             bool all_inputs_available = true;
//             switch(p) {
//             case VALUES:
//             case FORWARD:
//                 for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
//                     if(to_be_processed[graph.source(arc)]) {
//                         all_inputs_available = false;
//                         break;
//                     }
//                 }
//                 break;
//             case REVERSE:
//                 for(out_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
//                     if(to_be_processed[graph.target(arc)]) {
//                         all_inputs_available = false;
//                         break;
//                     }
//                 }
//                 break;
//             default:
//                 DEBUG_DEAD_LINE;
//             }
//             if(all_inputs_available) {
//                 to_be_processed[node] = false;
//                 new_processed.insert(node);
//                 double val = evaluate_node(node, p);
//                 DEBUG_OUT(3, "                All inputs available --> compute value (" << val << ")");
//                 switch(p) {
//                 case VALUES:
//                 case FORWARD:
//                     for(out_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
//                         node_t target_node = graph.target(arc);
//                         new_active_nodes.insert(target_node);
//                         DEBUG_OUT(4, "                    add '" << node_labels[target_node] << "' to active set");
//                     }
//                     break;
//                 case REVERSE:
//                     for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
//                         node_t source_node = graph.source(arc);
//                         new_active_nodes.insert(source_node);
//                         DEBUG_OUT(4, "                    add '" << node_labels[source_node] << "' to active set");
//                     }
//                     break;
//                 default:
//                     DEBUG_DEAD_LINE;
//                 }
//             } else {
//                 DEBUG_OUT(3, "                Some input pending --> move node to pending set");
//                 move_to_pending.insert(node);
//             }
//         }
//         //-------------//
//         // update sets //
//         //-------------//
//         // swap old and new active
//         active_nodes.swap(new_active_nodes);
//         // remove processed nodes from active and pending (this may also include
//         // new active nodes that were already processed in the same iteration)
//         DEBUG_OUT(4,"    Processed nodes:" << (new_processed.size()==0?" NONE":""));
//         for(node_t node : new_processed) {
//             DEBUG_OUT(4,"        " << node_labels[node]);
//             active_nodes.erase(node);
//             pending_nodes.erase(node);
//         }
//         // print new active nodes
//         IF_DEBUG(4) {
//             DEBUG_OUT(4,"    New active nodes:" << (active_nodes.size()==0?" NONE":""));
//             for(node_t node : active_nodes) {
//                 DEBUG_OUT(4,"        " << node_labels[node]);
//             }
//         }
//         DEBUG_OUT(4,"    Move to pending:" << (move_to_pending.size()==0?" NONE":""));
//         for(node_t node : move_to_pending) {
//             DEBUG_OUT(4,"        " << node_labels[node]);
//             pending_nodes.insert(node);
//         }
//     }
//     if(pending_nodes.size()>0 || active_nodes.size()>0) {
//         DEBUG_ERROR("Propagating values did not complete");
//     }
// }

void CG::propagate_values(TYPE p, std::vector<node_t> changed_nodes) {

    switch(p) {
    case VALUES: {
        DEBUG_OUT(1, "Propagating values...");
        auto prop = GraphPropagationFactory(graph);
        for(node_t node : changed_nodes) prop.add_source(node);
        prop.init();
        for(node_t node=prop.next(); node!=lemon::INVALID; node=prop.next()) {
            double val = evaluate_node(node, p);
            DEBUG_OUT(3, "    compute value for " << node_labels[node] << ": " << val);
        }
        break;
    }
    case FORWARD: {
        DEBUG_OUT(1, "Propagating differentials (forward)...");
        auto prop = GraphPropagationFactory(graph);
        for(node_t node : changed_nodes) prop.add_source(node);
        prop.init();
        for(node_t node=prop.next(); node!=lemon::INVALID; node=prop.next()) {
            double val = evaluate_node(node, p);
            DEBUG_OUT(3, "    compute value for " << node_labels[node] << ": " << val);
        }
        break;
    }
    case REVERSE: {
        DEBUG_OUT(1, "Propagating differentials (reverse)...");
        auto prop = GraphPropagationFactory(reverseDigraph(graph));
        for(node_t node : changed_nodes) prop.add_source(node);
        prop.init();
        for(node_t node=prop.next(); node!=lemon::INVALID; node=prop.next()) {
            double val = evaluate_node(node, p);
            DEBUG_OUT(3, "    compute value for " << node_labels[node] << ": " << val);
        }
        break;
    }
    default:
        DEBUG_DEAD_LINE;
    }
}

CG & CG::compute_values(std::vector<double> values) {
    update_values(input_nodes, values);
    return *this;
}

CG & CG::forward_accumulation(std::vector<double> values,
                              std::vector<double> differentials) {
    compute_values(values);
    update_differentials_forward(input_nodes, differentials);
    return *this;
}

CG & CG::reverse_accumulation(std::vector<double> values,
                              std::vector<double> differentials) {
    compute_values(values);
    update_differentials_reverse(output_nodes, differentials);
    return *this;
}

CG & CG::update_values(std::vector<node_t> nodes,
                       std::vector<double> values,
                       bool input_nodes_only) {
    if(input_nodes_only and !includes(input_nodes, nodes)) {
        DEBUG_WARNING("Given nodes are not a subset of the input nodes.");
        return *this;
    }
    if(!values.empty()) {
        assign_values(values, nodes);
    }
    propagate_values(VALUES, nodes);
    return *this;
}

CG & CG::update_differentials_forward(std::vector<node_t> nodes,
                                      std::vector<double> differentials,
                                      bool input_nodes_only) {
    if(input_nodes_only and !includes(input_nodes, nodes)) {
        DEBUG_WARNING("Given nodes are not a subset of the input nodes.");
        return *this;
    }
    if(!differentials.empty()) {
        assign_differentials(differentials, nodes);
    }
    propagate_values(FORWARD, nodes);
    return *this;
}

CG & CG::update_differentials_reverse(std::vector<node_t> nodes,
                                      std::vector<double> differentials,
                                      bool output_nodes_only) {
    if(output_nodes_only and !includes(output_nodes, nodes)) {
        DEBUG_WARNING("Given nodes are not a subset of the output nodes.");
        return *this;
    }
    if(!differentials.empty()) {
        assign_differentials(differentials, nodes);
    }
    propagate_values(REVERSE, nodes);
    return *this;
}

bool CG::check_derivatives(std::vector<double> values,
                           double delta,
                           double epsilon_absolute,
                           double epsilon_relative) {
    // compute all values
    compute_values(values);
    // make a copy of the derivatives
    graph_t::ArcMap<double> arc_values_copy(graph);
    mapCopy(graph, arc_values, arc_values_copy);
    // change the value of all nodes by delta and nummerically check derivatives
    bool ok = true;
    for(node_it_t node(graph); node!=INVALID; ++node) {
        DEBUG_OUT(2,"Checking derivatives w.r.t " <<
                  node_labels[node] << " (id" << graph.id(node) << ")");
        // get dependent nodes (the ones we need to check)
        vector<node_t> dependent_nodes;
        vector<arc_t> dependent_arcs;
        for(out_arc_it_t arc(graph, node); arc!=INVALID; ++arc) {
            dependent_nodes.push_back(graph.target(arc));
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
        DEBUG_EXPECT(1, (int)dependent_nodes.size()==num_vars &&
                     (int)dependent_arcs.size()==num_vars &&
                     (int)values_1.size()==num_vars &&
                     (int)values_2.size()==num_vars);
        for(int idx : Range(num_vars)) {
            DEBUG_OUT(2,"Checking derivative of " << node_labels[dependent_nodes[idx]] <<
                      " (id" << graph.id(dependent_nodes[idx]) << ") w.r.t " <<
                      node_labels[node] << " (id" << graph.id(node) << ")");
            double numerical_derivative = (values_1[idx] - values_2[idx])/delta;
            double analytical_derivative = arc_values_copy[dependent_arcs[idx]];
            if(fabs(numerical_derivative-analytical_derivative)>epsilon_absolute and
               fabs(numerical_derivative/analytical_derivative)>1+epsilon_relative) {
                ok = false;
                DEBUG_WARNING("Partial derivative of " << node_labels[dependent_nodes[idx]] <<
                              " (id" << graph.id(dependent_nodes[idx]) << ") w.r.t " <<
                              node_labels[node] <<
                              " (id" << graph.id(node) << ") is out of bounds (numerical=" <<
                              numerical_derivative << " / analytical=" <<
                              analytical_derivative << ")");
            }
        }
    }
    return ok;
}

CG & CG::plot_graph(const char* file_name) {
    graph_t::NodeMap<QString> node_properties(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        //node_properties[node] = QString("label=<%1=%2<BR/><I>d</I>%1=%3>").
        node_properties[node] = QString("label=<%1=%2<BR/><I>d</I>%1=%3<BR/>id=%4>").
            arg(node_labels[node]).
            arg(node_values[node]).
            arg(node_differentials[node]).
            arg(graph.id(node));
    }
    graph_t::ArcMap<QString> arc_properties(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        arc_properties[arc] = QString("label=<%1>").
            arg(arc_values[arc]);
    }
    util::graph_to_pdf(file_name, graph, "", &node_properties, "", &arc_properties);
    return *this;
}

double CG::evaluate_node(const node_t & node, TYPE e) {
    switch(e) {
    case VALUES:
    {
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
            if(input_size!=(int)variables.size()) {
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
    case FORWARD:
    {
        // evaluate arcs
        double val = 0;
        for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
            val += node_differentials[graph.source(arc)]*arc_values[arc];
        }
        node_differentials[node] = val;
        return val;
    }
    case REVERSE:
    {
        // evaluate arcs
        double val = 0;
        for(out_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
            val += node_differentials[graph.target(arc)]*arc_values[arc];
        }
        node_differentials[node] = val;
        return val;
    }
    default:
        DEBUG_DEAD_LINE;
        return 0;
    }
}

CG & CG::set_node(node_t node,
                  QString label,
                  std::vector<QString> variables,
                  function_t function) {
    node_labels[node] = label;
    node_values[node] = NAN;
    node_differentials[node] = NAN;
    node_variables[node] = variables;
    node_functions[node] = function;
    return *this;
}

CG & CG::set_arc(arc_t arc,
                 function_t function) {
    arc_values[arc] = NAN;
    arc_functions[arc] = function;
    return *this;
}

vector<double> CG::get_input_differentials() const {
    vector<double> values;
    for(node_t node : input_nodes) {
        values.push_back(node_differentials[node]);
    }
    return values;
}

vector<double> CG::get_output_differentials() const {
    vector<double> values;
    for(node_t node : output_nodes) {
        values.push_back(node_differentials[node]);
    }
    return values;
}

vector<double> CG::get_output_values() const {
    vector<double> values;
    for(node_t node : output_nodes) {
        values.push_back(node_values[node]);
    }
    return values;
}

CG & CG::set_input_nodes(std::vector<node_t> n) {
    input_nodes = n;
    return *this;
}

CG & CG::set_output_nodes(std::vector<node_t> n) {
    output_nodes = n;
    return *this;
}

bool CG::check_graph_structure(bool reset_input_nodes,
                               bool reset_output_nodes) {
    bool ok = true;

    //==================//
    // check for cycles //
    //==================//
    if(!dag(graph)) {
        DEBUG_WARNING("Graph contains cycles");
        ok = false;
    }

    //==========================//
    // check input/output nodes //
    //==========================//
    set<node_t> new_input_nodes, new_output_nodes;
    for(node_it_t node(graph); node!=INVALID; ++node) {
        if(in_arc_it_t(graph,node)==INVALID) new_input_nodes.insert(node);
        if(out_arc_it_t(graph,node)==INVALID) new_output_nodes.insert(node);
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

    //======================//
    // Check variable lists //
    //======================//
    for(node_it_t node(graph); node!=INVALID; ++node) {
        // get actual labels by scanning incomming arcs/nodes
        set<QString> actual_labels;
        for(in_arc_it_t arc(graph,node); arc!=INVALID; ++arc) {
            QString new_label = node_labels[graph.source(arc)];
            if(actual_labels.find(new_label)!=actual_labels.end()) {
                DEBUG_WARNING("Found duplicate label '" << new_label <<
                              "' in nodes adjacent to node '" << node_labels[node] << "'");
                ok = false;
            } else {
                actual_labels.insert(new_label);
            }
        }
        // get provided labels (also check for duplicates)
        set<QString> provided_labels;
        for(QString new_label : node_variables[node]) {
            if(provided_labels.find(new_label)!=provided_labels.end()) {
                DEBUG_WARNING("Found duplicate label '" << new_label <<
                              "' in provided labels of node '" << node_labels[node] << "'");
                ok = false;
            } else {
                provided_labels.insert(new_label);
            }
        }
        // check if labels are the same
        if(actual_labels!=provided_labels) {
            DEBUG_WARNING("For node '" << node_labels[node] <<
                          "' actual labels (from adjacent nodes) and provided labels (on construction) do not match.");
            DEBUG_WARNING("      actual: " << util::container_to_str(actual_labels,"' '","'","'"));
            DEBUG_WARNING("    provided: " << util::container_to_str(provided_labels,"' '","'","'"));
        }
    }

    return ok;
}

QString CG::get_node_label(node_t node) const {
    return node_labels[node];
}

double CG::get_node_value(node_t node) const {
    return node_values[node];
}

double CG::get_node_differential(node_t node) const {
    return node_differentials[node];
}

double CG::get_arc_value(arc_t arc) const {
    return arc_values[arc];
}

void CG::set_node_label(node_t node, QString label) {
    node_labels[node] = label;
}

void CG::set_node_value(node_t node, double value) {
    node_values[node] = value;
}

void CG::set_node_function(node_t node, std::vector<QString> variables, function_t function) {
    node_variables[node] = variables;
    node_functions[node] = function;
}

void CG::set_node_differential(node_t node, double value) {
    node_differentials[node] = value;
}

void CG::set_arc_value(arc_t arc, double value) {
    arc_values[arc] = value;
}

bool CG::includes(std::vector<node_t> nodes, std::vector<node_t> subnodes) {
    set<node_t> node_set(nodes.begin(), nodes.end());
    set<node_t> subnode_set(subnodes.begin(), subnodes.end());
    return std::includes(
        node_set.begin(), node_set.end(),
        subnode_set.begin(), subnode_set.end()
        );
}
