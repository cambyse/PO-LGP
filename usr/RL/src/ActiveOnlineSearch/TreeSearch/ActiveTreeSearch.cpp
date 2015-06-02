#include "ActiveTreeSearch.h"

#include "NodeFinder.h"

#include <util/util.h>
#include <util/return_tuple.h>
#include <util/graph_plotting.h>

#include <typeinfo>
#include <sstream>
#include <set>
#include <vector>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using lemon::INVALID;
using std::vector;
using std::set;

ActiveTreeSearch::ActiveTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                                   double discount,
                                   std::shared_ptr<node_finder::NodeFinder> node_finder):
    SearchTree(environment,discount,node_finder),
    variable_info_map(graph),
    base_node(c_graph),
    c_node_name(c_graph),
    counts(graph),
    computer(c_graph)
{
    if(typeid(*node_finder)!=typeid(node_finder::PlainTree)) {
        DEBUG_WARNING("Using ActiveTreeSearch with a NodeFinder other than PlainTree is not tested");
    }
}

void ActiveTreeSearch::next_do() {
    environment->set_state(root_state);
    bool go_on = true;
    node_t current_observation_node = root_node;
    set<node_t> action_nodes_to_update;
    DEBUG_OUT(1,"Next...");
    while(go_on) {
        // make a transition
        auto action = util::random_select(environment->get_actions());
        RETURN_TUPLE(observation_handle_t, observation,
                     reward_t, reward ) = environment->transition(action);
        DEBUG_OUT(1,"    (action --> observation) = (" << *action << " --> " << *observation << ")");
        // find/add action node
        RETURN_TUPLE(arc_t, to_action_arc,
                     node_t, action_node,
                     bool, created_to_action_arc,
                     bool, created_action_node) = find_or_create_action_node(current_observation_node,
                                                                             action);
        // find/add observation node
        RETURN_TUPLE(arc_t, to_observation_arc,
                     node_t, observation_node,
                     bool, created_to_observation_arc,
                     bool, created_observation_node) = find_or_create_observation_node(action_node,
                                                                                       observation);
        // increment counts (init if created)
        if(created_to_action_arc) counts[to_action_arc] = 0;
        if(created_to_observation_arc) counts[to_observation_arc] = 0;
        ++counts[to_action_arc];
        ++counts[to_observation_arc];
        // update
        action_nodes_to_update.insert(action_node);
        current_observation_node = observation_node;
        go_on = !created_observation_node;
    }
    DEBUG_OUT(1,"...expanded tree");

    // update node values
    for(node_t node : action_nodes_to_update) {
        update_c_node_values(node);
    }

    // propagate values
    {
        vector<node_t> nodes;
        for(node_it_t node(c_graph); node!=INVALID; ++node) {
            nodes.push_back(node);
        }
        computer.check_graph_structure(true,true); // recompute input and output nodes
        computer.compute_values();
    }
}

ActiveTreeSearch::action_handle_t ActiveTreeSearch::recommend_action() const {
    return *(environment->get_actions().begin());
}

void ActiveTreeSearch::init(const state_handle_t & root_state) {
    SearchTree::init(root_state);
    c_graph.clear();
    c_root_node = c_graph.addNode();
    base_node[c_root_node] = root_node;
    c_node_name[c_root_node] = "root";
}

void ActiveTreeSearch::toPdf(const char* file_name) const {
    const bool use_id = false;
    graph_t combi_graph;
    graph_t::NodeMap<node_t> graph_map(graph);
    graph_t::NodeMap<node_t> c_graph_map(c_graph);
    graph_t::NodeMap<QString> node_prop_map(combi_graph);
    graph_t::ArcMap<QString> arc_prop_map(combi_graph);
    // copy graph nodes
    for(node_it_t node(graph); node!=INVALID; ++node) {
        node_t combi_node = combi_graph.addNode();
        graph_map[node] = combi_node;
        if(use_id) {
            node_prop_map[combi_node] = QString("shape=%1 label=<%2<BR/>id=%3>").
                arg(node_info_map[node].type==OBSERVATION_NODE?"square":"circle").
                arg(str_html(node)).
                arg(graph.id(node));
        } else {
            node_prop_map[combi_node] = QString("shape=%1 label=<%2>").
                arg(node_info_map[node].type==OBSERVATION_NODE?"square":"circle").
                arg(str_html(node));
        }
    }
    // copy c_graph nodes
    for(node_it_t c_node(c_graph); c_node!=INVALID; ++c_node) {
        node_t combi_node = combi_graph.addNode();
        c_graph_map[c_node] = combi_node;
        QString color;
        if(c_node_name[c_node].startsWith("MeanQ_") || c_node_name[c_node].startsWith("VarQ_") || c_node_name[c_node].startsWith("pi_")) {
            color = ".3 .2 1";
        } else if(c_node_name[c_node].startsWith("MeanR_") || c_node_name[c_node].startsWith("VarR_") ||
                  c_node_name[c_node].startsWith("MeanP_") || c_node_name[c_node].startsWith("VarP_")) {
            color = "1 .2 1";
        } else if(c_node_name[c_node]=="A" || c_node_name[c_node]=="B" || c_node_name[c_node]=="C" ||
                  c_node_name[c_node].startsWith("alpha_") || c_node_name[c_node].startsWith("beta_") || c_node_name[c_node].startsWith("gamma_")) {
            color = ".6 .2 1";
        } else {
            color = "1 1 1";
        }
        if(use_id) {
            node_prop_map[combi_node] = QString("shape=%1 style=\"%2\" label=<%3<BR/>%6/%7<BR/>id=%5> color=\"%4\"").
                arg("box").
                arg("rounded,filled").
                arg(c_node_name[c_node]).
                arg(color).
                arg(c_graph.id(c_node)).
                arg(computer.get_node_value(c_node)).
                arg(computer.get_node_differential(c_node));
        } else {
            node_prop_map[combi_node] = QString("shape=%1 style=\"%2\" label=<%3<BR/>%5/%6> color=\"%4\"").
                arg("box").
                arg("rounded,filled").
                arg(c_node_name[c_node]).
                arg(color).
                arg(computer.get_node_value(c_node)).
                arg(computer.get_node_differential(c_node));
        }
    }
    // copy graph arcs
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        arc_t combi_arc = combi_graph.addArc(graph_map[graph.source(arc)],
                                             graph_map[graph.target(arc)]);
        if(node_info_map[graph.source(arc)].type==OBSERVATION_NODE) {
            arc_prop_map[combi_arc] = QString("style=dashed label=<%1>").arg(counts[arc]);
        } else {
            arc_prop_map[combi_arc] = QString("style=solid label=<%1>").arg(counts[arc]);
        }
    }
    // copy c_graph arcs
    for(arc_it_t arc(c_graph); arc!=INVALID; ++arc) {
        // note: reverse direction (target->source instead of source->target)
        // for cleaner plotting but then re-reverse so that arrow heads are at
        // the correct end in the plot
        arc_t combi_arc = combi_graph.addArc(c_graph_map[c_graph.target(arc)],
                                             c_graph_map[c_graph.source(arc)]);
        arc_prop_map[combi_arc] = QString("dir=back style=solid color=\".3 .8 .8\"");
    }
    // add associating arcs
    for(node_it_t c_node(c_graph); c_node!=INVALID; ++c_node) {
        arc_t combi_arc = combi_graph.addArc(graph_map[base_node[c_node]],c_graph_map[c_node]);
        arc_prop_map[combi_arc] = QString("dir=none style=dotted color=\"1 .7 .7\"");
    }
    // write to file
    util::graph_to_pdf(file_name,
                       combi_graph,
                       "style=filled truecolor=true",
                       &node_prop_map,
                       "",
                       &arc_prop_map,
                       true,
                       "dot");
}

ActiveTreeSearch::arc_node_t ActiveTreeSearch::add_observation_node(observation_handle_t observation,
                                                                    node_t action_node) {
    auto return_value = SearchTree::add_observation_node(observation,action_node);
    RETURN_TUPLE(arc_t,arc, node_t,observation_node, bool,dummy1, bool,dummy2) = return_value;

    // get observation description
    QString observation_str;
    {
        std::stringstream ss;
        observation->write(ss);
        observation_str = ss.str().c_str();
    }

    // add c_nodes (arrays)
    variable_info_map[action_node].mean_p[observation_node] = c_graph.addNode();
    variable_info_map[action_node].alpha[ observation_node] = c_graph.addNode();
    variable_info_map[action_node].beta[  observation_node] = c_graph.addNode();
    // remember base node (arrays)
    base_node[variable_info_map[action_node].mean_p[observation_node]] = action_node;
    base_node[variable_info_map[action_node].alpha[ observation_node]] = action_node;
    base_node[variable_info_map[action_node].beta[  observation_node]] = action_node;
    // set name (arrays)
    c_node_name[variable_info_map[action_node].mean_p[observation_node]] = QString("MeanP_%1").arg(observation_str);
    c_node_name[variable_info_map[action_node].alpha[ observation_node]] = QString("alpha_%1").arg(observation_str);
    c_node_name[variable_info_map[action_node].beta[  observation_node]] = QString("beta_%1" ).arg(observation_str);
    // add arcs (arrays)
    c_graph.addArc(variable_info_map[action_node].alpha[ observation_node], variable_info_map[action_node].A);
    c_graph.addArc(variable_info_map[action_node].beta[  observation_node], variable_info_map[action_node].B);
    c_graph.addArc(variable_info_map[action_node].mean_p[observation_node], variable_info_map[action_node].A);
    c_graph.addArc(variable_info_map[action_node].mean_p[observation_node], variable_info_map[action_node].B);

    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t other_observation_node = graph.target(arc);
        // get other_observation description
        QString other_observation_str;
        {
            std::stringstream ss;
            observation->write(ss);
            other_observation_str = ss.str().c_str();
        }

        // observation_node --> other_observation_node
        {
            // add c_nodes (matrices)
            variable_info_map[action_node].var_p[observation_node][other_observation_node] = c_graph.addNode();
            variable_info_map[action_node].gamma[observation_node][other_observation_node] = c_graph.addNode();
            // remember base node (matrices)
            base_node[variable_info_map[action_node].var_p[observation_node][other_observation_node]] = action_node;
            base_node[variable_info_map[action_node].gamma[observation_node][other_observation_node]] = action_node;
            // set name (matrices)
            c_node_name[variable_info_map[action_node].var_p[observation_node][other_observation_node]] = QString("VarP_%1_%2").arg(observation_str).arg(other_observation_str);
            c_node_name[variable_info_map[action_node].gamma[observation_node][other_observation_node]] = QString("gamma_%1_%2").arg(observation_str).arg(other_observation_str);
            // add arcs (matrices)
            c_graph.addArc(variable_info_map[action_node].var_p[observation_node][other_observation_node], variable_info_map[action_node].C);
            c_graph.addArc(variable_info_map[action_node].gamma[observation_node][other_observation_node], variable_info_map[action_node].C);
        }
        // other_observation_node --> observation_node (only if not equal)
        if(observation_node!=other_observation_node) {
            // add c_nodes (matrices)
            variable_info_map[action_node].var_p[other_observation_node][observation_node] = c_graph.addNode();
            variable_info_map[action_node].gamma[other_observation_node][observation_node] = c_graph.addNode();
            // remember base node (matrices)
            base_node[variable_info_map[action_node].var_p[other_observation_node][observation_node]] = action_node;
            base_node[variable_info_map[action_node].gamma[other_observation_node][observation_node]] = action_node;
            // set name (matrices)
            c_node_name[variable_info_map[action_node].var_p[other_observation_node][observation_node]] = QString("VarP_%1_%2").arg(other_observation_str).arg(observation_str);
            c_node_name[variable_info_map[action_node].gamma[other_observation_node][observation_node]] = QString("gamma_%1_%2").arg(other_observation_str).arg(observation_str);
            // add arcs (matrices)
            c_graph.addArc(variable_info_map[action_node].var_p[other_observation_node][observation_node], variable_info_map[action_node].C);
            c_graph.addArc(variable_info_map[action_node].gamma[other_observation_node][observation_node], variable_info_map[action_node].C);
        } else {
            // if they are equal (the covariance then is the variance) we have
            // to add another connection
            c_graph.addArc(variable_info_map[action_node].var_p[observation_node][observation_node], variable_info_map[action_node].B);
        }
    }
    update_c_node_connections(action_node);

    return return_value;
}

ActiveTreeSearch::arc_node_t ActiveTreeSearch::add_action_node(action_handle_t action,
                                                               node_t observation_node) {
    auto return_value = SearchTree::add_action_node(action, observation_node);
    RETURN_TUPLE(arc_t,arc, node_t,action_node, bool,dummy1, bool,dummy2) = return_value;

    // get action description
    QString action_str;
    {
        std::stringstream ss;
        action->write(ss);
        action_str = ss.str().c_str();
    }

    set<node_t> nodes_to_update;

    // add c_nodes
    variable_info_map[action_node].pi     = c_graph.addNode();
    variable_info_map[action_node].mean_Q = c_graph.addNode();
    variable_info_map[action_node].var_Q  = c_graph.addNode();
    variable_info_map[action_node].mean_r = c_graph.addNode();
    variable_info_map[action_node].var_r  = c_graph.addNode();
    variable_info_map[action_node].A      = c_graph.addNode();
    variable_info_map[action_node].B      = c_graph.addNode();
    variable_info_map[action_node].C      = c_graph.addNode();
    // remember base node
    base_node[variable_info_map[action_node].pi]     = action_node;
    base_node[variable_info_map[action_node].mean_Q] = action_node;
    base_node[variable_info_map[action_node].var_Q]  = action_node;
    base_node[variable_info_map[action_node].mean_r] = action_node;
    base_node[variable_info_map[action_node].var_r]  = action_node;
    base_node[variable_info_map[action_node].A]      = action_node;
    base_node[variable_info_map[action_node].B]      = action_node;
    base_node[variable_info_map[action_node].C]      = action_node;
    // set name
    c_node_name[variable_info_map[action_node].pi]     = QString("pi_%1").arg(action_str);
    c_node_name[variable_info_map[action_node].mean_Q] = QString("MeanQ_%1").arg(action_str);
    c_node_name[variable_info_map[action_node].var_Q]  = QString("VarQ_%1").arg(action_str);
    c_node_name[variable_info_map[action_node].mean_r] = QString("MeanR_%1").arg(action_str);
    c_node_name[variable_info_map[action_node].var_r]  = QString("VarR_%1").arg(action_str);
    c_node_name[variable_info_map[action_node].A]      = "A";
    c_node_name[variable_info_map[action_node].B]      = "B";
    c_node_name[variable_info_map[action_node].C]      = "C";
    // add arcs
    c_graph.addArc(variable_info_map[action_node].mean_r,variable_info_map[action_node].mean_Q);
    c_graph.addArc(variable_info_map[action_node].A,variable_info_map[action_node].mean_Q);
    c_graph.addArc(variable_info_map[action_node].var_r,variable_info_map[action_node].var_Q);
    c_graph.addArc(variable_info_map[action_node].B,variable_info_map[action_node].var_Q);
    c_graph.addArc(variable_info_map[action_node].C,variable_info_map[action_node].var_Q);

    // add arcs for policy
    for(out_arc_it_t arc(graph,observation_node); arc!=INVALID; ++arc) {
        node_t other_action_node = graph.target(arc);
        nodes_to_update.insert(other_action_node);
        c_graph.addArc(variable_info_map[action_node].mean_Q,variable_info_map[other_action_node].pi);
        c_graph.addArc(variable_info_map[action_node].var_Q,variable_info_map[other_action_node].pi);
        if(other_action_node!=action_node) {
            c_graph.addArc(variable_info_map[other_action_node].mean_Q,variable_info_map[action_node].pi);
            c_graph.addArc(variable_info_map[other_action_node].var_Q,variable_info_map[action_node].pi);
        }
    }

    // connect to ancestor action node
    in_arc_it_t arc_from_ancestor_action_node(graph,observation_node);
    if(arc_from_ancestor_action_node!=INVALID) {
        node_t ancestor_action_node = graph.source(arc_from_ancestor_action_node);
        nodes_to_update.insert(ancestor_action_node);
        c_graph.addArc(variable_info_map[action_node].pi,     variable_info_map[ancestor_action_node].alpha[observation_node]);
        c_graph.addArc(variable_info_map[action_node].mean_Q, variable_info_map[ancestor_action_node].alpha[observation_node]);
        c_graph.addArc(variable_info_map[action_node].pi,     variable_info_map[ancestor_action_node].beta[observation_node]);
        c_graph.addArc(variable_info_map[action_node].var_Q,  variable_info_map[ancestor_action_node].beta[observation_node]);
        for(auto p : variable_info_map[ancestor_action_node].gamma[observation_node]) {
            node_t other_observation_node = p.first;
            node_t gamma_node = p.second;
            c_graph.addArc(variable_info_map[action_node].var_Q, gamma_node);
            if(other_observation_node!=observation_node) {
                c_graph.addArc(variable_info_map[action_node].var_Q, variable_info_map[ancestor_action_node].gamma[other_observation_node][observation_node]);
            }
        }
    } else {
        c_graph.addArc(variable_info_map[action_node].pi, c_root_node);
        update_c_root_connections();
    }

    // update
    for(node_t node : nodes_to_update) {
        update_c_node_connections(node);
    }

    return return_value;
}

void ActiveTreeSearch::erase_node(node_t node) {
    SearchTree::erase_node(node);
    DEBUG_WARNING("Erasing nodes is not properly implemented");
}

void ActiveTreeSearch::update_c_node_connections(node_t action_node) {
    DEBUG_EXPECT(0,node_info_map[action_node].type==ACTION_NODE);

    for(node_t c_node : {
                variable_info_map[action_node].pi,
                variable_info_map[action_node].mean_Q,
                variable_info_map[action_node].var_Q,
                variable_info_map[action_node].mean_r,
                variable_info_map[action_node].var_r,
                variable_info_map[action_node].A,
                variable_info_map[action_node].B,
                variable_info_map[action_node].C
                    }) {
        vector<QString> input_names;
        for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
            input_names.push_back(c_node_name[c_graph.source(arc)]);
        }
        computer.set_node(c_node,
                          c_node_name[c_node],
                          input_names,
                          [](vector<double> v)->double{return v.size();});
        for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
            computer.set_arc(arc, [](vector<double>)->double{return 0;});
        }
    }

    for(node_array_t array : {
                variable_info_map[action_node].mean_p,
                variable_info_map[action_node].alpha,
                variable_info_map[action_node].beta
                    }) {
        for(auto c_node_pair : array) {
            node_t c_node = c_node_pair.second;
            vector<QString> input_names;
            for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                input_names.push_back(c_node_name[c_graph.source(arc)]);
            }
            computer.set_node(c_node,
                              c_node_name[c_node],
                              input_names,
                              [](vector<double> v)->double{return v.size();});
            for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                computer.set_arc(arc, [](vector<double>)->double{return 0;});
            }
        }
    }

    for(node_matrix_t matrix : {
                variable_info_map[action_node].var_p,
                variable_info_map[action_node].gamma
                    }) {
        for(auto array : matrix) {
            for(auto c_node_pair : array.second) {
                node_t c_node = c_node_pair.second;
                vector<QString> input_names;
                for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                    input_names.push_back(c_node_name[c_graph.source(arc)]);
                }
                computer.set_node(c_node,
                                  c_node_name[c_node],
                                  input_names,
                                  [](vector<double> v)->double{return v.size();});
                for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                    computer.set_arc(arc, [](vector<double>)->double{return 0;});
                }
            }
        }
    }
}

void ActiveTreeSearch::update_c_node_values(node_t action_node) {
    for(node_t c_node : {
                variable_info_map[action_node].pi,
                variable_info_map[action_node].mean_Q,
                variable_info_map[action_node].var_Q,
                variable_info_map[action_node].mean_r,
                variable_info_map[action_node].var_r,
                variable_info_map[action_node].A,
                variable_info_map[action_node].B,
                variable_info_map[action_node].C
                }) {
        vector<QString> input_names;
        for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
            input_names.push_back(c_node_name[c_graph.source(arc)]);
        }
        computer.set_node_value(c_node,0);
    }

    for(node_array_t array : {
                variable_info_map[action_node].mean_p,
                variable_info_map[action_node].alpha,
                variable_info_map[action_node].beta
                }) {
        for(auto c_node_pair : array) {
            node_t c_node = c_node_pair.second;
            vector<QString> input_names;
            for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                input_names.push_back(c_node_name[c_graph.source(arc)]);
            }
            computer.set_node_value(c_node,0);
        }
    }
    for(node_matrix_t matrix : {
                variable_info_map[action_node].var_p,
                variable_info_map[action_node].gamma
                }) {
        for(auto array : matrix) {
            for(auto c_node_pair : array.second) {
                node_t c_node = c_node_pair.second;
                vector<QString> input_names;
                for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                    input_names.push_back(c_node_name[c_graph.source(arc)]);
                }
                computer.set_node_value(c_node,0);
            }
        }
    }
}

void ActiveTreeSearch::update_c_root_connections() {
    vector<QString> input_names;
    for(in_arc_it_t arc(c_graph,c_root_node); arc!=INVALID; ++arc) {
        input_names.push_back(c_node_name[c_graph.source(arc)]);
    }
    computer.set_node(c_root_node,
                      c_node_name[c_root_node],
                      input_names,
                      [](vector<double> v)->double{return v.size();});
    for(in_arc_it_t arc(c_graph,c_root_node); arc!=INVALID; ++arc) {
        computer.set_arc(arc, [](vector<double>)->double{return 0;});
    }
}
