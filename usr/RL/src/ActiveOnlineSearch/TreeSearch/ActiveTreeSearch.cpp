#include "ActiveTreeSearch.h"

#include "NodeFinder.h"

#include <util/util.h>
#include <util/QtUtil.h>
#include <util/pretty_printer.h>
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
using std::map;
using std::numeric_limits;
using util::container_to_str;

ActiveTreeSearch::ActiveTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                                   double discount,
                                   std::shared_ptr<node_finder::NodeFinder> node_finder):
    SearchTree(environment,discount,node_finder),
    variable_info_map(graph),
    associated_state(graph),
    base_node(c_graph),
    counts(graph),
    reward_sum(graph),
    reward_square_sum(graph),
    computer(c_graph)
{
    if(typeid(*node_finder)!=typeid(node_finder::PlainTree)) {
        DEBUG_WARNING("Using ActiveTreeSearch with a NodeFinder other than PlainTree is not tested");
    }
}

void ActiveTreeSearch::next_do() {
    environment->set_state(root_state);
    set<node_t> action_nodes_to_update;
    DEBUG_OUT(1,"Next...");
    for(int i=0; i<1; ++i) {
        // choose observation node at random and set environment's state
        vector<node_t> candidate_nodes;
        for(node_it_t node(graph); node!=INVALID; ++node) {
            if(node_info_map[node].type==ACTION_NODE) continue;
            // // determine depth
            // int depth = 0;
            // node_t ancestor = node;
            // while(ancestor!=INVALID) {
            //     in_arc_it_t arc(graph,ancestor);
            //     if(arc!=INVALID) {
            //         ancestor = graph.source(arc);
            //         ++depth;
            //     } else {
            //         ancestor = INVALID;
            //     }
            // }
            // if(depth/2>3) continue;
            candidate_nodes.push_back(node);
        }
        node_t old_observation_node = INVALID;
        while(old_observation_node==INVALID) {
            old_observation_node = util::random_select(candidate_nodes);
            environment->set_state(associated_state[old_observation_node]);
            if(environment->is_terminal_state()) {
                DEBUG_OUT(1,"    Discard observation node:");
                if(old_observation_node==root_node) {
                    DEBUG_OUT(1,"        root");
                } else {
                    DEBUG_OUT(1,"        " << *(node_info_map[old_observation_node].observation));
                }
                old_observation_node=INVALID;
            } else {
                DEBUG_OUT(1,"    Choose observation node:");
                if(old_observation_node==root_node) {
                    DEBUG_OUT(1,"        root");
                } else {
                    DEBUG_OUT(1,"        " << *(node_info_map[old_observation_node].observation));
                }
            }
        }
        // make a random transition
        auto action = util::random_select(environment->get_actions());
        RETURN_TUPLE(observation_handle_t, observation,
                     reward_t, reward ) = environment->transition(action);
        DEBUG_OUT(1,"    (action --> observation) = (" << *action << " --> " << *observation << ")");
        // find/add action node
        RETURN_TUPLE(arc_t, to_action_arc,
                     node_t, action_node,
                     bool, created_to_action_arc,
                     bool, created_action_node) = find_or_create_action_node(old_observation_node,
                                                                             action);
        // find/add observation node
        RETURN_TUPLE(arc_t, to_observation_arc,
                     node_t, new_observation_node,
                     bool, created_to_observation_arc,
                     bool, created_observation_node) = find_or_create_observation_node(action_node,
                                                                                       observation);
        // increment counts and rewards (init if created)
        if(created_to_observation_arc) counts[to_observation_arc] = 0;
        if(created_to_action_arc) {
            counts[to_action_arc] = 0;
            reward_sum[to_action_arc] = 0;
            reward_square_sum[to_action_arc] = 0;
        }
        ++counts[to_observation_arc];
        ++counts[to_action_arc];
        reward_sum[to_action_arc] += reward;
        reward_square_sum[to_action_arc] += reward*reward;
        // update
        associated_state[new_observation_node] = environment->get_state_handle();
        action_nodes_to_update.insert(action_node);
    }
    DEBUG_OUT(1,"...expanded tree");

    // update node values
    for(node_t node : action_nodes_to_update) {
        update_c_node_values(node);
    }

    // propagate values through computational graph
    {
        vector<node_t> nodes;
        for(node_it_t node(c_graph); node!=INVALID; ++node) {
            nodes.push_back(node);
        }
        computer.check_graph_structure(true,true); // identify input and output nodes
        computer.check_derivatives();
        //computer.compute_values();
        computer.set_node_differential(c_root_node,1);
        computer.reverse_accumulation();
    }
}

ActiveTreeSearch::action_handle_t ActiveTreeSearch::recommend_action() const {
    return *(environment->get_actions().begin());
}

void ActiveTreeSearch::init(const state_handle_t & root_state) {
    SearchTree::init(root_state);
    associated_state[root_node] = root_state;
    c_graph.clear();
    c_root_node = c_graph.addNode();
    base_node[c_root_node] = root_node;
    computer.set_node_label(c_root_node, "root");
}

void ActiveTreeSearch::toPdf(const char* file_name) const {
    const bool use_id = true;
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
        if(computer.get_node_label(c_node).startsWith("MeanQ_") ||
           computer.get_node_label(c_node).startsWith("VarQ_") ||
           computer.get_node_label(c_node).startsWith("pi_")) {
            color = ".3 .2 1";
        } else if(computer.get_node_label(c_node).startsWith("MeanR_") ||
                  computer.get_node_label(c_node).startsWith("VarR_") ||
                  computer.get_node_label(c_node).startsWith("MeanP_") ||
                  computer.get_node_label(c_node).startsWith("VarP_")) {
            color = "1 .2 1";
        } else if(computer.get_node_label(c_node)=="A" ||
                  computer.get_node_label(c_node)=="B" ||
                  computer.get_node_label(c_node)=="C" ||
                  computer.get_node_label(c_node).startsWith("alpha_") ||
                  computer.get_node_label(c_node).startsWith("beta_") ||
                  computer.get_node_label(c_node).startsWith("gamma_")) {
            color = ".6 .2 1";
        } else {
            color = "1 1 1";
        }
        if(use_id) {
            node_prop_map[combi_node] = QString("shape=%1 style=\"%2\" label=<%3<BR/>%6/%7<BR/>id=%5> color=\"%4\"").
                arg("box").
                arg("rounded,filled").
                arg(computer.get_node_label(c_node)).
                arg(color).
                arg(c_graph.id(c_node)).
                arg(computer.get_node_value(c_node)).
                arg(computer.get_node_differential(c_node));
        } else {
            node_prop_map[combi_node] = QString("shape=%1 style=\"%2\" label=<%3<BR/>%5/%6> color=\"%4\"").
                arg("box").
                arg("rounded,filled").
                arg(computer.get_node_label(c_node)).
                arg(color).
                arg(computer.get_node_value(c_node)).
                arg(computer.get_node_differential(c_node));
        }
    }
    // copy graph arcs
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        arc_t combi_arc = combi_graph.addArc(graph_map[graph.source(arc)],
                                             graph_map[graph.target(arc)]);
        if(node_info_map[graph.source(arc)].type==ACTION_NODE) {
            arc_prop_map[combi_arc] = QString("style=solid label=<#%1>").arg(counts[arc]);
        } else {
            arc_prop_map[combi_arc] = QString("style=dashed label=<#%1<BR/>%2/%3>").
                arg(counts[arc]).
                arg(reward_sum[arc]).
                arg(reward_square_sum[arc]);
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
    QString observation_str = description(observation);

    // mean_p, alpha, beta
    {
        // add nodes
        variable_info_map[action_node].mean_p[observation_node] = c_graph.addNode();
        variable_info_map[action_node].alpha[ observation_node] = c_graph.addNode();
        variable_info_map[action_node].beta[  observation_node] = c_graph.addNode();
        // remember base node
        base_node[variable_info_map[action_node].mean_p[observation_node]] = action_node;
        base_node[variable_info_map[action_node].alpha[ observation_node]] = action_node;
        base_node[variable_info_map[action_node].beta[  observation_node]] = action_node;
        // set label
        computer.set_node_label(variable_info_map[action_node].mean_p[observation_node],
                                QString("MeanP_%1").arg(observation_str));
        computer.set_node_label(variable_info_map[action_node].alpha[ observation_node],
                                QString("alpha_%1").arg(observation_str));
        computer.set_node_label(variable_info_map[action_node].beta[  observation_node],
                                QString("beta_%1" ).arg(observation_str));
        // add connecting arcs
        c_graph.addArc(variable_info_map[action_node].alpha[ observation_node], variable_info_map[action_node].A);
        c_graph.addArc(variable_info_map[action_node].beta[  observation_node], variable_info_map[action_node].B);
        c_graph.addArc(variable_info_map[action_node].mean_p[observation_node], variable_info_map[action_node].A);
        c_graph.addArc(variable_info_map[action_node].mean_p[observation_node], variable_info_map[action_node].B);
    }

    // var_p
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t other_observation_node = graph.target(arc);
        QString other_observation_str = description(node_info_map[other_observation_node].observation);
        // observation_node --> other_observation_node
        {
            // add node
            variable_info_map[action_node].var_p[observation_node][other_observation_node] = c_graph.addNode();
            // remember base node
            base_node[variable_info_map[action_node].var_p[observation_node][other_observation_node]] = action_node;
            // set label
            computer.set_node_label(variable_info_map[action_node].var_p[observation_node][other_observation_node],
                                    QString("VarP_%1_%2").arg(observation_str).arg(other_observation_str));
            // add arcs
            c_graph.addArc(variable_info_map[action_node].var_p[observation_node][other_observation_node],
                           variable_info_map[action_node].C);
        }
        if(observation_node==other_observation_node) {
            // another arc for the variance
            c_graph.addArc(variable_info_map[action_node].var_p[observation_node][observation_node], variable_info_map[action_node].B);
            continue; // don't add symmetric nodes/arcs twice
        }
        // other_observation_node --> observation_node
        {
            // add node
            variable_info_map[action_node].var_p[other_observation_node][observation_node] = c_graph.addNode();
            // remember base node
            base_node[variable_info_map[action_node].var_p[other_observation_node][observation_node]] = action_node;
            // set label
            computer.set_node_label(variable_info_map[action_node].var_p[other_observation_node][observation_node],
                                    QString("VarP_%1_%2").arg(other_observation_str).arg(observation_str));
            // add arc
            c_graph.addArc(variable_info_map[action_node].var_p[other_observation_node][observation_node],
                           variable_info_map[action_node].C);
        }
    }

    // gamma
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t other_observation_node = graph.target(arc);
        QString other_observation_str = description(node_info_map[other_observation_node].observation);
        // observation_node --> other_observation_node
        {
            // add nodes
            variable_info_map[action_node].gamma[observation_node][other_observation_node] = c_graph.addNode();
            // remember base node
            base_node[variable_info_map[action_node].gamma[observation_node][other_observation_node]] = action_node;
            // set label
            computer.set_node_label(variable_info_map[action_node].gamma[observation_node][other_observation_node],
                                    QString("gamma_%1_%2").arg(observation_str).arg(other_observation_str));
            // add arc
            c_graph.addArc(variable_info_map[action_node].gamma[observation_node][other_observation_node],
                           variable_info_map[action_node].C);
        }
        if(observation_node==other_observation_node) continue;
        // other_observation_node --> observation_node (only if not equal)
        {
            // add node
            variable_info_map[action_node].gamma[other_observation_node][observation_node] = c_graph.addNode();
            // remember base node
            base_node[variable_info_map[action_node].gamma[other_observation_node][observation_node]] = action_node;
            // set label
            computer.set_node_label(variable_info_map[action_node].gamma[other_observation_node][observation_node],
                                    QString("gamma_%1_%2").arg(other_observation_str).arg(observation_str));
            // add arc
            c_graph.addArc(variable_info_map[action_node].gamma[other_observation_node][observation_node], variable_info_map[action_node].C);
        }
        // find inputs from already existing nodes
        for(out_arc_it_t arc(graph,other_observation_node); arc!=INVALID; ++arc) {
            node_t other_action_node = graph.target(arc);
            node_t gamma_1 = variable_info_map[action_node].gamma[other_observation_node][observation_node];
            node_t gamma_2 = variable_info_map[action_node].gamma[observation_node][other_observation_node];
            c_graph.addArc(variable_info_map[other_action_node].pi, gamma_1);
            c_graph.addArc(variable_info_map[other_action_node].pi, gamma_2);
            c_graph.addArc(variable_info_map[other_action_node].mean_Q, gamma_1);
            c_graph.addArc(variable_info_map[other_action_node].mean_Q, gamma_2);
        }
    }

    // update connections
    update_c_node_connections(action_node);

    return return_value;
}

ActiveTreeSearch::arc_node_t ActiveTreeSearch::add_action_node(action_handle_t action,
                                                               node_t observation_node) {
    auto return_value = SearchTree::add_action_node(action, observation_node);
    RETURN_TUPLE(arc_t,arc, node_t,action_node, bool,dummy1, bool,dummy2) = return_value;

    // get descriptions
    QString action_str = description(action);
    QString observation_str = description(node_info_map[observation_node].observation);

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
    computer.set_node_label(variable_info_map[action_node].pi,
                            QString("pi_%1_|_%2").arg(action_str).arg(observation_str));
    computer.set_node_label(variable_info_map[action_node].mean_Q,
                            QString("MeanQ_%1_%2").arg(observation_str).arg(action_str));
    computer.set_node_label(variable_info_map[action_node].var_Q,
                            QString("VarQ_%1_%2").arg(observation_str).arg(action_str));
    computer.set_node_label(variable_info_map[action_node].mean_r,
                            QString("MeanR_%1").arg(action_str));
    computer.set_node_label(variable_info_map[action_node].var_r,
                            QString("VarR_%1").arg(action_str));
    computer.set_node_label(variable_info_map[action_node].A,
                            "A");
    computer.set_node_label(variable_info_map[action_node].B,
                            "B");
    computer.set_node_label(variable_info_map[action_node].C,
                            "C");
    // add simple arcs (between variables both belonging to this node)
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
        // alpha, beta
        c_graph.addArc(variable_info_map[action_node].pi,     variable_info_map[ancestor_action_node].alpha[observation_node]);
        c_graph.addArc(variable_info_map[action_node].mean_Q, variable_info_map[ancestor_action_node].alpha[observation_node]);
        c_graph.addArc(variable_info_map[action_node].pi,     variable_info_map[ancestor_action_node].beta[observation_node]);
        c_graph.addArc(variable_info_map[action_node].var_Q,  variable_info_map[ancestor_action_node].beta[observation_node]);
        // gamma
        for(out_arc_it_t arc(graph,ancestor_action_node); arc!=INVALID; ++arc) {
            node_t other_observation_node = graph.target(arc);
            node_t gamma_node_1 = variable_info_map[ancestor_action_node].gamma[observation_node][other_observation_node];
            c_graph.addArc(variable_info_map[action_node].pi, gamma_node_1);
            c_graph.addArc(variable_info_map[action_node].mean_Q, gamma_node_1);
            if(other_observation_node==observation_node) continue;
            node_t gamma_node_2 = variable_info_map[ancestor_action_node].gamma[other_observation_node][observation_node];
            c_graph.addArc(variable_info_map[action_node].pi, gamma_node_2);
            c_graph.addArc(variable_info_map[action_node].mean_Q, gamma_node_2);
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

    // pi
    {
        // construct input list
        vector<QString> input_names;
        int this_idx = -1;
        node_t observation_node = graph.source(in_arc_it_t(graph,action_node));
        {
            int idx = 0;
            for(out_arc_it_t arc(graph,observation_node); arc!=INVALID; ++arc) {
                node_t other_action_node = graph.target(arc);
                if(other_action_node==action_node) this_idx = idx;
                input_names.push_back(computer.get_node_label(variable_info_map[other_action_node].mean_Q));
                input_names.push_back(computer.get_node_label(variable_info_map[other_action_node].var_Q));
                ++idx;
            }
        }
        DEBUG_EXPECT(0,this_idx!=-1);
        // method for computing the policy
        const double tau = 1;
        const double Cp = 1;
        node_t c_node = variable_info_map[action_node].pi;
        auto policy = [tau,Cp](vector<double> v, int a_idx){
            vector<reward_t> upper_bounds;
            bool is_inf = false;
            int inf_values = 0;
            for(int idx=0; idx<(int)v.size(); idx+=2) {
                reward_t upper = v[idx]+Cp*v[idx+1];
                if((numeric_limits<reward_t>::has_infinity &&
                    upper==numeric_limits<reward_t>::infinity()) ||
                   upper==numeric_limits<reward_t>::max()) {
                    ++inf_values;
                    if(idx/2==a_idx) is_inf = true;
                }
                upper_bounds.push_back(upper);
            }
            if(inf_values>0) {
                return is_inf?1./inf_values:0;
            } else {
                return util::soft_max(upper_bounds,tau)[a_idx];
            }
        };
        // set node function
        computer.set_node_function(c_node,
                                   input_names,
                                   [policy,this_idx](vector<double> v){return policy(v,this_idx);});
        // set arc functions
        int idx = 0;
        for(out_arc_it_t arc(graph,observation_node); arc!=INVALID; ++arc) {
            node_t other_action_node = graph.target(arc);
            arc_t mean_Q_arc = lemon::findArc(c_graph,variable_info_map[other_action_node].mean_Q,c_node);
            arc_t var_Q_arc = lemon::findArc(c_graph,variable_info_map[other_action_node].var_Q,c_node);
            DEBUG_EXPECT(0,mean_Q_arc!=INVALID);
            DEBUG_EXPECT(0,var_Q_arc!=INVALID);
            if(other_action_node==action_node) {
                DEBUG_EXPECT(0,idx==this_idx);
                computer.set_arc(mean_Q_arc,
                                 [tau,policy,this_idx](vector<double> v){
                                     return (1-policy(v,this_idx))*policy(v,this_idx)/tau;
                                 });
                computer.set_arc(var_Q_arc,
                                 [tau,policy,this_idx](vector<double> v){
                                     return (1-policy(v,this_idx))*policy(v,this_idx)/tau;
                                 });
            } else {
                DEBUG_EXPECT(0,idx!=this_idx);
                computer.set_arc(mean_Q_arc,
                                 [tau,policy,idx,this_idx](vector<double> v){
                                     return -policy(v,idx)*policy(v,this_idx)/tau;
                                 });
                computer.set_arc(var_Q_arc,
                                 [tau,policy,idx,this_idx](vector<double> v){
                                     return -policy(v,idx)*policy(v,this_idx)/tau;
                                 });
            }
            ++idx;
        }
    }

    // mean_Q
    {
        node_t c_node = variable_info_map[action_node].mean_Q;
        computer.set_node_function(c_node,
                                   {computer.get_node_label(variable_info_map[action_node].mean_r),
                                           computer.get_node_label(variable_info_map[action_node].A)},
                                   [](vector<double> v){
                                       return v[0]+v[1];
                                   });
        for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
            computer.set_arc(arc, [](vector<double>){return 1;});
        }
    }

    // var_Q
    {
        node_t c_node = variable_info_map[action_node].var_Q;
        computer.set_node_function(c_node,
                                   {computer.get_node_label(variable_info_map[action_node].var_r),
                                           computer.get_node_label(variable_info_map[action_node].B),
                                           computer.get_node_label(variable_info_map[action_node].C)},
                                   [](vector<double> v){
                                       return v[0]+v[1]+v[2];
                                   });
        for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
            computer.set_arc(arc, [](vector<double>){return 1;});
        }
    }

    // A, B
    vector<QString> A_input_list, B_input_list;
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t s_prime_prime = graph.target(arc);
        A_input_list.push_back(computer.get_node_label(variable_info_map[action_node].mean_p[s_prime_prime]));
        A_input_list.push_back(computer.get_node_label(variable_info_map[action_node].alpha[s_prime_prime]));
        B_input_list.push_back(computer.get_node_label(variable_info_map[action_node].mean_p[s_prime_prime]));
        B_input_list.push_back(computer.get_node_label(variable_info_map[action_node].var_p[s_prime_prime][s_prime_prime]));
        B_input_list.push_back(computer.get_node_label(variable_info_map[action_node].beta[s_prime_prime]));
    }
    {
        node_t c_node = variable_info_map[action_node].A;
        computer.set_node_function(c_node,
                                   A_input_list,
                                   [this](vector<double> v){
                                       double sum = 0;
                                       for(int idx=0; idx<(int)v.size(); idx+=2) {
                                           sum += discount*v[idx]*v[idx+1];
                                       }
                                       return sum;
                                   });
        int idx = 0;
        for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
            node_t s_prime_prime = graph.target(arc);
            arc_t mean_p_arc = lemon::findArc(c_graph,variable_info_map[action_node].mean_p[s_prime_prime],c_node);
            arc_t alpha_arc = lemon::findArc(c_graph,variable_info_map[action_node].alpha[s_prime_prime],c_node);
            computer.set_arc(mean_p_arc, [this,idx](vector<double> v){return discount*v[2*idx+1];});
            computer.set_arc(alpha_arc, [this,idx](vector<double> v){return discount*v[2*idx];});
            ++idx;
        }
    }
    {
        node_t c_node = variable_info_map[action_node].B;
        computer.set_node_function(c_node,
                                   B_input_list,
                                   [this](vector<double> v){
                                       double sum = 0;
                                       for(int idx=0; idx<(int)v.size(); idx+=3) {
                                           sum += discount*(pow(v[idx],2)+v[idx+1])*v[idx+2];
                                       }
                                       return sum;
                                   });
        int idx = 0;
        for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
            node_t s_prime_prime = graph.target(arc);
            arc_t mean_p_arc = lemon::findArc(c_graph,variable_info_map[action_node].mean_p[s_prime_prime],c_node);
            arc_t var_p_arc = lemon::findArc(c_graph,variable_info_map[action_node].var_p[s_prime_prime][s_prime_prime],c_node);
            arc_t beta_arc = lemon::findArc(c_graph,variable_info_map[action_node].beta[s_prime_prime],c_node);
            computer.set_arc(mean_p_arc, [this,idx](vector<double> v){return discount*2*v[2*idx]*v[2*idx+2];});
            computer.set_arc(var_p_arc, [this,idx](vector<double> v){return discount*v[2*idx+2];});
            computer.set_arc(beta_arc, [this,idx](vector<double> v){return discount*(pow(v[2*idx],2)+v[2*idx+1]);});
            ++idx;
        }
    }

    // alpha, beta
    map<node_t,vector<QString>> alpha_input_list, beta_input_list;
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t s_prime_prime = graph.target(arc);
        for(out_arc_it_t arc(graph,s_prime_prime); arc!=INVALID; ++arc) {
            node_t a_prime_prime = graph.target(arc);
            alpha_input_list[s_prime_prime].push_back(computer.get_node_label(variable_info_map[a_prime_prime].pi));
            alpha_input_list[s_prime_prime].push_back(computer.get_node_label(variable_info_map[a_prime_prime].mean_Q));
            beta_input_list[s_prime_prime].push_back(computer.get_node_label(variable_info_map[a_prime_prime].pi));
            beta_input_list[s_prime_prime].push_back(computer.get_node_label(variable_info_map[a_prime_prime].var_Q));
        }
    }
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t s_prime_prime = graph.target(arc);
        node_t c_node = variable_info_map[action_node].alpha[s_prime_prime];
        computer.set_node_function(c_node,
                                   alpha_input_list[s_prime_prime],
                                   [](vector<double> v){
                                       double sum = 0;
                                       for(int idx=0; idx<(int)v.size(); idx+=2) {
                                           sum += v[idx]*v[idx+1];
                                       }
                                       return sum;
                                   });
        for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
            computer.set_arc(arc, [](vector<double>){return 1;});
        }
    }
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t s_prime_prime = graph.target(arc);
        node_t c_node = variable_info_map[action_node].beta[s_prime_prime];
        computer.set_node_function(c_node,
                                   beta_input_list[s_prime_prime],
                                   [](vector<double> v){
                                       double sum = 0;
                                       for(int idx=0; idx<(int)v.size(); idx+=2) {
                                           sum += v[idx]*v[idx+1];
                                       }
                                       return sum;
                                   });
        for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
            computer.set_arc(arc, [](vector<double>){return 1;});
        }
    }

    // C
    vector<QString> C_input_list;
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t s_prime_prime = graph.target(arc);
        for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
            node_t s_prime_prime_prime = graph.target(arc);
            C_input_list.push_back(
                computer.get_node_label(variable_info_map[action_node].var_p[s_prime_prime][s_prime_prime_prime])
                );
            C_input_list.push_back(
                computer.get_node_label(variable_info_map[action_node].gamma[s_prime_prime][s_prime_prime_prime])
                );
        }
    }
    {
        node_t c_node = variable_info_map[action_node].C;
        computer.set_node_function(c_node,
                                   C_input_list,
                                   [this](vector<double> v){
                                       double sum = 0;
                                       for(int idx=0; idx<(int)v.size(); idx+=2) {
                                           sum += discount*v[idx]*v[idx+1];
                                       }
                                       return sum;
                                   });
        for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
            computer.set_arc(arc, [](vector<double>){return 1;});
        }
    }

    // gamma
    map<node_t,map<node_t,vector<QString>>> gamma_input_list;
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t s_prime_prime = graph.target(arc);
        for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
            node_t s_prime_prime_prime = graph.target(arc);
            auto & input_list = gamma_input_list[s_prime_prime][s_prime_prime_prime];
            for(out_arc_it_t arc(graph,s_prime_prime); arc!=INVALID; ++arc) {
                node_t a_prime_prime = graph.target(arc);
                input_list.push_back(computer.get_node_label(variable_info_map[a_prime_prime].pi));
                input_list.push_back(computer.get_node_label(variable_info_map[a_prime_prime].mean_Q));
            }
            if(s_prime_prime_prime==s_prime_prime) continue; // don't add inputs twice
            for(out_arc_it_t arc(graph,s_prime_prime_prime); arc!=INVALID; ++arc) {
                node_t a_prime_prime_prime = graph.target(arc);
                input_list.push_back(computer.get_node_label(variable_info_map[a_prime_prime_prime].pi));
                input_list.push_back(computer.get_node_label(variable_info_map[a_prime_prime_prime].mean_Q));
            }
        }
    }
    for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
        node_t s_prime_prime = graph.target(arc);
        for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
            node_t s_prime_prime_prime = graph.target(arc);
            node_t c_node = variable_info_map[action_node].gamma[s_prime_prime][s_prime_prime_prime];
            // Each gamma node (for a pair of observation nodes) takes four
            // input nodes for each action pair. If for one of the observation
            // nodes are ARE available actions but for the other NOT then there
            // are incomming arcs (in the computational graph) but nothing can
            // actually be computed because the other values are missing and the
            // input list (constructed above) is empty. We need to handle this
            // special case separately.
            if(gamma_input_list[s_prime_prime][s_prime_prime_prime].empty() && in_arc_it_t(c_graph,c_node)!=INVALID) {
#define FORCE_DEBUG_LEVEL 2
                DEBUG_OUT(1,"clear " << computer.get_node_label(c_node) << " (id=" << c_graph.id(c_node) << ")");
                for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                    gamma_input_list[s_prime_prime][s_prime_prime_prime].push_back(
                        computer.get_node_label(c_graph.source(arc)));

                }
                computer.set_node_function(c_node,
                                           gamma_input_list[s_prime_prime][s_prime_prime_prime],
                                           [](vector<double> v){return 0;});
                for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                    computer.set_arc(arc, [](vector<double>){return 0;});
                }
#define FORCE_DEBUG_LEVEL 2
            } else {
                computer.set_node_function(c_node,
                                           gamma_input_list[s_prime_prime][s_prime_prime_prime],
                                           [](vector<double> v){
                                               double sum = 0;
                                               for(int idx_1=0; idx_1<(int)v.size(); idx_1+=2) {
                                                   for(int idx_2=0; idx_2<(int)v.size(); idx_2+=2) {
                                                       sum += v[idx_1]*v[idx_1+1]*v[idx_2]*v[idx_2+1];
                                                   }
                                               }
                                               return sum;
                                           });
                for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                    computer.set_arc(arc, [](vector<double>){return 1;});
                }
            }
        }
    }

    //==================================================================//

    // update values (which were invalidated above)
    update_c_node_values(action_node);
}

void ActiveTreeSearch::update_c_node_values(node_t action_node) {
    // independent variables

    // needed for all of them
    arc_t to_action_arc = in_arc_it_t(graph,action_node);
    int this_counts = counts[to_action_arc];

    // mean_r, var_r
    {
        reward_t this_reward_sum = reward_sum[to_action_arc];
        reward_t this_reward_square_sum = reward_square_sum[to_action_arc];
        computer.set_node_value(variable_info_map[action_node].mean_r, this_reward_sum/this_counts);
        reward_t var;
        if(this_counts>1) {
            var = (this_reward_square_sum/this_counts - pow(this_reward_sum/this_counts,2)); // biased
            var *= (reward_t)this_counts/(this_counts-1); // unbiased
        } else {
            if(numeric_limits<reward_t>::has_infinity) {
                var = numeric_limits<reward_t>::infinity();
            } else {
                var = numeric_limits<reward_t>::max();
            }
        }
        computer.set_node_value(variable_info_map[action_node].var_r, var);
    }

    // mean_p
    {
#define FORCE_DEBUG_LEVEL 0
        DEBUG_OUT(1,"Compute mean_p");
        for(out_arc_it_t to_observation_arc(graph,action_node);
            to_observation_arc!=INVALID;
            ++to_observation_arc) {
            node_t observation_node = graph.target(to_observation_arc);
            DEBUG_EXPECT(0,node_info_map[observation_node].type==OBSERVATION_NODE);
            double mean_p = counts[to_observation_arc];
            mean_p /= this_counts;
            DEBUG_OUT(1,"    for action node " << graph.id(action_node) <<
                      " / observation node" << graph.id(observation_node));
            DEBUG_OUT(1,"        counts to observation=" << counts[to_observation_arc]);
            DEBUG_OUT(1,"        counts to action=" << this_counts);
            DEBUG_OUT(1,"        mean_p=" << mean_p);
#define FORCE_DEBUG_LEVEL 0
            computer.set_node_value(variable_info_map[action_node].mean_p[observation_node], mean_p);
        }
    }

    // var_p
    for(node_matrix_t matrix : {
                variable_info_map[action_node].var_p,
                }) {
        for(auto array : matrix) {
            for(auto c_node_pair : array.second) {
                node_t c_node = c_node_pair.second;
                computer.set_node_value(c_node,1);
            }
        }
    }

    // dependent variables
    for(node_t c_node : {
                variable_info_map[action_node].pi,
                variable_info_map[action_node].mean_Q,
                variable_info_map[action_node].var_Q,
                variable_info_map[action_node].A,
                variable_info_map[action_node].B,
                variable_info_map[action_node].C
                }) {
        // set to zero if no input available
        if(in_arc_it_t(c_graph,c_node)==INVALID) computer.set_node_value(c_node,0);
    }

    for(node_array_t array : {
                variable_info_map[action_node].alpha,
                variable_info_map[action_node].beta
                }) {
        for(auto c_node_pair : array) {
            node_t c_node = c_node_pair.second;
            // set to zero if no input available
            if(in_arc_it_t(c_graph,c_node)==INVALID) computer.set_node_value(c_node,0);
        }
    }
    for(node_matrix_t matrix : {
                variable_info_map[action_node].gamma
                }) {
        for(auto array : matrix) {
            for(auto c_node_pair : array.second) {
                node_t c_node = c_node_pair.second;
                // set to zero if no input available
                if(in_arc_it_t(c_graph,c_node)==INVALID) computer.set_node_value(c_node,0);
            }
        }
    }
}

void ActiveTreeSearch::update_c_root_connections() {
    vector<QString> input_names;
    for(in_arc_it_t arc(c_graph,c_root_node); arc!=INVALID; ++arc) {
        input_names.push_back(computer.get_node_label(c_graph.source(arc)));
    }
    computer.set_node_function(c_root_node,
                               input_names,
                               [](vector<double> v){
                                   double ret = 0;
                                   for(double pi : v) {
                                       if(pi==0) continue;
                                       ret += -pi*log(pi);
                                   }
                                   return ret;
                               });
    int idx = 0;
    for(in_arc_it_t arc(c_graph,c_root_node); arc!=INVALID; ++arc) {
        computer.set_arc(arc, [idx](vector<double> v){return 1 + log(v[idx]);});
        ++idx;
    }
}

QString ActiveTreeSearch::description(const action_handle_t action) {
    if(action==nullptr) {
        return "NULL";
    } else {
        std::stringstream ss;
        action->write(ss);
        return ss.str().c_str();
    }
}

QString ActiveTreeSearch::description(const observation_handle_t observation) {
    if(observation==nullptr) {
        return "NULL";
    } else {
        std::stringstream ss;
        observation->write(ss);
        return ss.str().c_str();
    }
}
