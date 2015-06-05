#include "MonteCarloTreeSearch.h"

#include "TreePolicy.h"
#include "ValueHeuristic.h"
#include "BackupMethod.h"

#include <vector>
#include <tuple>
#include <functional>
#include <memory>

#include <lemon/adaptors.h> // reverseDigraph()

#include "../graph_util.h"
#include <util/QtUtil.h>
#include <util/util.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <util/return_tuple.h>
#include <util/return_tuple_macros.h>

using lemon::INVALID;
using std::tuple;
using std::make_tuple;
using std::vector;
using std::shared_ptr;

int MonteCarloTreeSearch::MCTSNodeInfo::get_transition_counts() const {
    return transition_counts;
}

int MonteCarloTreeSearch::MCTSNodeInfo::get_rollout_counts() const {
    return rollout_counts;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSNodeInfo::get_value() const {
    return value;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSNodeInfo::get_value_variance() const {
    return value_variance;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSNodeInfo::get_return_sum() const {
    return return_sum;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSNodeInfo::get_squared_return_sum() const {
    return squared_return_sum;
}

void MonteCarloTreeSearch::MCTSNodeInfo::set_value(reward_t val, reward_t val_variance) {
    value=val;
    value_variance=val_variance;
}

void MonteCarloTreeSearch::MCTSNodeInfo::add_rollout_return(reward_t ret) {
    ++rollout_counts;
    return_sum+=ret;
    squared_return_sum+=ret*ret;
}

void MonteCarloTreeSearch::MCTSNodeInfo::add_transition() {
    ++transition_counts;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSNodeInfo::get_rollout_return_sum() const {
    return return_sum;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSNodeInfo::get_squared_rollout_return_sum() const {
    return squared_return_sum;
}

int MonteCarloTreeSearch::MCTSArcInfo::get_rollout_counts() const {
    return rollout_counts;
}

int MonteCarloTreeSearch::MCTSArcInfo::get_transition_counts() const {
    return transition_counts;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSArcInfo::get_reward_sum() const {
    return reward_sum;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSArcInfo::get_squared_reward_sum() const {
    return squared_reward_sum;
}

void MonteCarloTreeSearch::MCTSArcInfo::add_rollout_return(reward_t reward) {
    ++rollout_counts;
    reward_sum+=reward;
    squared_reward_sum+=reward*reward;
}

void MonteCarloTreeSearch::MCTSArcInfo::add_transition() {
    ++transition_counts;
}

MonteCarloTreeSearch::MonteCarloTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                                           double discount,
                                           std::shared_ptr<node_finder::NodeFinder> node_finder,
                                           std::shared_ptr<tree_policy::TreePolicy> tree_policy,
                                           std::shared_ptr<value_heuristic::ValueHeuristic> value_heuristic,
                                           std::shared_ptr<backup_method::BackupMethod> backup_method,
                                           BACKUP_TYPE backup_type,
                                           int max_depth):
    SearchTree(environment, discount, node_finder),
    mcts_node_info_map(graph),
    mcts_arc_info_map(graph),
    tree_policy(tree_policy),
    value_heuristic(value_heuristic),
    backup_method(backup_method),
    backup_type(backup_type),
    node_hash(graph),
    max_depth(max_depth)
{
    tree_policy->init(environment,graph,node_info_map,mcts_node_info_map,mcts_arc_info_map);
    value_heuristic->init(discount,environment);
    backup_method->init(discount,environment,graph,node_info_map,mcts_arc_info_map);
}

void MonteCarloTreeSearch::next_do() {
    // remember the trajectory
    typedef tuple<node_t,arc_t,node_t,arc_t,reward_t> trajectory_item_t;
    vector<trajectory_item_t> trajectory;

    /* ========================================================================
       (1) follow tree-policy to leaf-node (2) go one more step to expand this
       leaf node (3) stop if newly created node is a leaf or node or goto 1 if
       it is a non-leaf node (only in DAGs)
       ======================================================================== */
    node_t leaf_node = INVALID;
    bool is_real_leaf_node = true;
    {
        DEBUG_OUT(2,"Follow tree-policy...");
        node_t current_node = root_node;
        #warning XXXXX
        //environment->set_state(root_state);
        node_set_t node_set({root_node},0,node_hash);
        DEBUG_OUT(2,"    Starting at node " << graph.id(root_node));
        for(int depth=0; max_depth<0 || depth<max_depth; ++depth) {

            // get tree-policy action
            action_handle_t action = tree_policy->get_action(current_node);

            // find or create action node
            T(arc_t, to_action_arc,
              node_t, action_node,
              bool, new_action_arc,
              bool, new_action_node) = find_or_create_action_node(current_node, action);

            // perfrom transition
            T(observation_handle_t, observation_to, reward_t, reward) = environment->transition(action);

            // find or create observation node
            T(arc_t, to_observation_arc,
              node_t, observation_node,
              bool, new_observation_arc,
              bool, new_observation_node) = find_or_create_observation_node(action_node, observation_to);
            DEBUG_OUT(2,"    Transition:");
            DEBUG_OUT(2,"        action " << *action << " / node " << graph.id(action_node));
            DEBUG_OUT(2,"        observation " << *observation_to << " / node " << graph.id(observation_node));
            DEBUG_OUT(2,"        reward " << reward);

            // add to trajectory
            trajectory.push_back(trajectory_item_t(current_node, to_action_arc, action_node, to_observation_arc, reward));

            // update current node and state
            current_node = observation_node;

            // break in terminal nodes
            if(environment->is_terminal_state()) break;
            // break if new observation node was added
            if(new_observation_node) break;
            // break if a node was visited before during this rollout (this can
            // happen in the case of FullGraph node finder, the problem is that
            // the node data is not up-to-date because it depends on the current
            // rollout) or update node set
            if(node_set.find(current_node)!=node_set.end()) {
                is_real_leaf_node = false;
                break;
            } else {
                node_set.insert(current_node);
            }
        }
        DEBUG_OUT(2,(is_real_leaf_node?"...reached leaf-node":"...reached closed loop"));
        leaf_node = current_node;
    }

    /* =======================================
       get a heuristic value estimate
       ======================================= */
    if(is_real_leaf_node) {
        value_heuristic->add_value_estimate(leaf_node,
                                            mcts_node_info_map);
    }

    /* =======================================
       backpropagate
       ======================================= */

    DEBUG_OUT(2,"Backup nodes...");

    /* We need to propagate back the returns to allow MC backups. If backup_type
     * is BACKUP_TRACE we also do the backups here. In that case only nodes that
     * lie on the trajectory will be backed up. (In trees this is the only way
     * but in general BACKUP_PROPAGATE will backup more nodes.) */
    {
        // initialize discounted return of this rollout with leaf-node's return
        reward_t discounted_return = mcts_node_info_map[leaf_node].get_return_sum()/mcts_node_info_map[leaf_node].get_rollout_counts();
        // follow the trace back to root node
        node_t observation_node, action_node;
        arc_t to_action_arc, to_observation_arc;
        reward_t reward;
        for(auto transition=trajectory.rbegin(); transition!=trajectory.rend(); ++transition) {
            t(observation_node,to_action_arc,action_node,to_observation_arc,reward) = *transition;
            // calculate discounted return
            discounted_return = reward + discount*discounted_return;
            // update counts, reward, and return
            mcts_node_info_map[observation_node  ].add_rollout_return(discounted_return);
            mcts_node_info_map[action_node       ].add_rollout_return(discounted_return);
            mcts_node_info_map[observation_node  ].add_transition();
            mcts_node_info_map[action_node       ].add_transition();
            mcts_arc_info_map[to_action_arc      ].add_rollout_return(reward);
            mcts_arc_info_map[to_observation_arc ].add_rollout_return(reward);
            mcts_arc_info_map[to_action_arc      ].add_transition();
            mcts_arc_info_map[to_observation_arc ].add_transition();
            DEBUG_OUT(2,QString("    update observation-node(%1):	counts=%2/%3	return_sum=%4").
                      arg(graph.id(observation_node)).
                      arg(mcts_node_info_map[observation_node].get_transition_counts()).
                      arg(mcts_node_info_map[observation_node].get_rollout_counts()).
                      arg(mcts_node_info_map[observation_node].get_return_sum()));
            DEBUG_OUT(2,QString("    update action-node(%1):	counts=%2/%3	return_sum=%4").
                      arg(graph.id(action_node)).
                      arg(mcts_node_info_map[action_node].get_transition_counts()).
                      arg(mcts_node_info_map[action_node].get_rollout_counts()).
                      arg(mcts_node_info_map[action_node].get_return_sum()));
            DEBUG_OUT(2,QString("    reward=%1, return=%2").arg(reward).arg(discounted_return));
            // backup if back_type is BACKUP_TRACE
            if(backup_type==BACKUP_TRACE) {
                backup_method->backup(observation_node,
                                      action_node,
                                      mcts_node_info_map);
            }
        }
    }

    if(backup_type==BACKUP_TRACE) {
        // backups were done above
    } else if(backup_type==BACKUP_PROPAGATE) {
        /* Backups will be propagated through the graph. */

        // construct and initialize graph propagation object
        auto graph_propagation = graph_util::GraphPropagationFactory(lemon::reverseDigraph(graph));
        graph_propagation.add_source(leaf_node).init();

        // process nodes
        for(node_t next_node = leaf_node;
            next_node!=INVALID;
            next_node = graph_propagation.next()) {

            // skip action nodes (will be backed up via observation nodes)
            if(node_info_map[next_node].type==ACTION_NODE) continue;

            // iterate through all observation-action pairs that can lead to
            // this observation node
            for(in_arc_it_t to_observation_arc(graph,next_node);
                to_observation_arc!=INVALID;
                ++to_observation_arc) {
                    node_t action_node = graph.source(to_observation_arc);
                    arc_t to_action_arc = in_arc_it_t(graph,action_node);
                    node_t observation_node = graph.source(to_action_arc);
                    backup_method->backup(observation_node,
                                          action_node,
                                          mcts_node_info_map);
                    DEBUG_OUT(2,QString("    update observation-node(%1):	counts=%2/%3	return_sum=%4").
                              arg(graph.id(observation_node)).
                              arg(mcts_node_info_map[observation_node].get_transition_counts()).
                              arg(mcts_node_info_map[observation_node].get_rollout_counts()).
                              arg(mcts_node_info_map[observation_node].get_return_sum()));
                    DEBUG_OUT(2,QString("    update action-node(%1):	counts=%2/%3	return_sum=%4").
                              arg(graph.id(action_node)).
                              arg(mcts_node_info_map[action_node].get_transition_counts()).
                              arg(mcts_node_info_map[action_node].get_rollout_counts()).
                              arg(mcts_node_info_map[action_node].get_return_sum()));
            }

        }
    } else DEBUG_DEAD_LINE;

    // backup root node (for global models)
    backup_method->backup_root(root_node,
                               mcts_node_info_map);
}


MonteCarloTreeSearch::action_handle_t MonteCarloTreeSearch::recommend_action() const {
    std::vector<action_handle_t> optimal_actions({*(environment->get_actions().begin())});
    double max_value = -DBL_MAX;
    for(out_arc_it_t arc(graph, root_node); arc!=INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        double value = mcts_node_info_map[action_node].get_value();
        if(value>max_value) {
            optimal_actions.clear();
            max_value = value;
        }
        if(value>=max_value) {
            optimal_actions.push_back(node_info_map[action_node].action);
        }
    }
    return util::random_select(optimal_actions);
}

void MonteCarloTreeSearch::toPdf(const char* file_name) const {

    //-----------------------------------------//
    // get min and max value/reward and counts //
    //-----------------------------------------//
    double min_val = DBL_MAX;
    double max_val = -DBL_MAX;
    int max_counts = 0;
    for(node_it_t node(graph); node!=INVALID; ++node) {
        min_val = std::min(min_val, mcts_node_info_map[node].get_value());
        max_val = std::max(max_val, mcts_node_info_map[node].get_value());
        max_counts = std::max(max_counts, mcts_node_info_map[node].get_transition_counts());
    }
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        if(node_info_map[graph.source(arc)].type==ACTION_NODE) {
            min_val = std::min(min_val, mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_rollout_counts());
            max_val = std::max(max_val, mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_rollout_counts());
        }
    }
    double norm = std::max(fabs(min_val), fabs(max_val));
    norm = norm==0?1:norm;

    graph_t::NodeMap<QString> node_map(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        double value = mcts_node_info_map[node].get_value();
        node_map[node] = QString("shape=%1 label=<%2<BR/>id=%5<BR/>#%3/%10<BR/>V=%4 +/- %11<BR/>R=%9> fillcolor=\"%6 %7 1\" penwidth=%8").
            //node_map[node] = QString("shape=%1 label=<%2<BR/>#%3/%10<BR/>V=%4 +/- %11<BR/>R=%9> fillcolor=\"%6 %7 1\" penwidth=%8").
            arg(node_info_map[node].type==OBSERVATION_NODE?"square":"circle").
            arg(str_html(node)).
            arg(mcts_node_info_map[node].get_transition_counts()).
            arg(value,0,'g',2).
            arg(graph.id(node)).
            arg(value>0?0.3:0).
            arg(color_rescale(fabs(value/norm))).
            arg(10.*mcts_node_info_map[node].get_transition_counts()/max_counts+0.1).
            arg(mcts_node_info_map[node].get_return_sum()).
            arg(mcts_node_info_map[node].get_rollout_counts()).
            arg(sqrt(mcts_node_info_map[node].get_value_variance()),0,'g',2);
    }

    graph_t::ArcMap<QString> arc_map(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        node_t source = graph.source(arc);
        double value = mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_rollout_counts();
        if(node_info_map[source].type==OBSERVATION_NODE) {
            arc_map[arc] = QString("style=dashed label=<#%1<BR/>r=%4> color=\"%2 %3 %3\"").
                arg(mcts_arc_info_map[arc].get_rollout_counts()).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm))).
                arg(mcts_arc_info_map[arc].get_reward_sum());
        } else {
            arc_map[arc] = QString("style=solid label=<#%2<BR/>r=%6> penwidth=%3 color=\"%4 %5 %5\"").
                arg(mcts_arc_info_map[arc].get_rollout_counts()).
                arg(5.*mcts_arc_info_map[arc].get_transition_counts()/mcts_node_info_map[source].get_transition_counts()+0.1).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm))).
                arg(mcts_arc_info_map[arc].get_reward_sum());
        }
    }

    util::graph_to_pdf(file_name,
                       graph,
                       "style=filled truecolor=true",
                       &node_map,
                       "",
                       &arc_map);
}

const MonteCarloTreeSearch::mcts_node_info_map_t & MonteCarloTreeSearch::get_mcts_node_info_map() const {
    return mcts_node_info_map;
}
const MonteCarloTreeSearch::mcts_arc_info_map_t & MonteCarloTreeSearch::get_mcts_arc_info_map() const {
    return mcts_arc_info_map;
}

double MonteCarloTreeSearch::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}
