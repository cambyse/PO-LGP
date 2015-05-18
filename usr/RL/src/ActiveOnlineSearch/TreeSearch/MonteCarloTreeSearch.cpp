#include "MonteCarloTreeSearch.h"

#include "TreePolicy.h"
#include "ValueHeuristic.h"
#include "BackupMethod.h"

#include <vector>
#include <tuple>
#include <functional>
#include <memory>

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
void MonteCarloTreeSearch::MCTSNodeInfo::add_separate_rollout(reward_t ret) {
    ++rollout_counts;
    return_sum+=ret;
    squared_return_sum+=ret*ret;
}
void MonteCarloTreeSearch::MCTSNodeInfo::add_rollout_on_trajectory(reward_t ret) {
    add_separate_rollout(ret);
    ++transition_counts;
}

void MonteCarloTreeSearch::MCTSNodeInfo::set_action_list(action_container_t & container) {
    DEBUG_EXPECT(0,unused_actions.size()==0);
    DEBUG_EXPECT(0,used_actions.size()==0);
    for(action_handle_t action : container) {
        unused_actions.insert(action);
    }
}

MonteCarloTreeSearch::action_handle_t MonteCarloTreeSearch::MCTSNodeInfo::use_random_action() {
    action_handle_t action = util::random_select(unused_actions);
    unused_actions.erase(action);
    used_actions.insert(action);
    return action;
}

bool MonteCarloTreeSearch::MCTSNodeInfo::is_fully_expanded() const {
    return unused_actions.size()==0;
}

int MonteCarloTreeSearch::MCTSArcInfo::get_counts() const {
    return counts;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSArcInfo::get_reward_sum() const {
    return reward_sum;
}

MonteCarloTreeSearch::reward_t MonteCarloTreeSearch::MCTSArcInfo::get_squared_reward_sum() const {
    return squared_reward_sum;
}

void MonteCarloTreeSearch::MCTSArcInfo::add_transition(reward_t reward) {
    ++counts;
    reward_sum+=reward;
    squared_reward_sum+=reward*reward;
}

MonteCarloTreeSearch::MonteCarloTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                                           double discount,
                                           std::shared_ptr<NodeFinder> node_finder,
                                           std::shared_ptr<tree_policy::TreePolicy> tree_policy,
                                           std::shared_ptr<value_heuristic::ValueHeuristic> value_heuristic,
                                           std::shared_ptr<backup_method::BackupMethod> backup_method,
                                           BACKUP_TYPE backup_type_):
    SearchTree(environment, discount, node_finder),
    mcts_node_info_map(graph),
    mcts_arc_info_map(graph),
    tree_policy(tree_policy),
    value_heuristic(value_heuristic),
    backup_method(backup_method),
    backup_type(backup_type_),
    node_hash(graph)
{}

void MonteCarloTreeSearch::next() {

    // remember the trajectory
    typedef tuple<node_t,arc_t,node_t,arc_t,reward_t> trajectory_item_t;
    vector<trajectory_item_t> trajectory;

    /* ========================================================================
       (1) follow tree-policy to leaf-node (2) go one more step to expand this
       leaf node (3) stop if newly created node is a leaf or node or goto 1 if
       it is a non-leaf node (only in DAGs)
       ======================================================================== */
    node_t leaf_node = INVALID;
    state_handle_t leaf_state = nullptr;
    {
        DEBUG_OUT(2,"Follow tree-policy...");
        node_t current_node = root_node;
        state_handle_t current_state = root_state;
        bool did_expansion = false;
        bool is_inner_node = mcts_node_info_map[current_node].is_fully_expanded();
        bool was_visited_before = false;
        while(is_inner_node || !did_expansion || was_visited_before) {

            // get tree-policy action
            action_handle_t action = (*tree_policy)(current_node,
                                                    environment,
                                                    graph,
                                                    node_info_map,
                                                    mcts_node_info_map,
                                                    mcts_arc_info_map);

            // find or create action node
            T(arc_t, to_action_arc, node_t, action_node) = find_or_create_action_node(current_node, action);

            // perfrom transition
            T(observation_handle_t, observation_to, reward_t, reward) = environment->transition(current_state, action);

            // find or create observation node
            T(arc_t, to_observation_arc, node_t, observation_node) = find_or_create_observation_node(action_node, observation_to);

            // add to trajectory
            trajectory.push_back(trajectory_item_t(current_node, to_action_arc, action_node, to_observation_arc, reward));

            // update node and state
            current_node = observation_node;
            current_state = environment->get_state_handle();

            // update halting conditions (last step was an expansion if the node was
            // not an inner node)
            did_expansion = !is_inner_node;
            is_inner_node = mcts_node_info_map[current_node].is_fully_expanded();
            if(environment->is_terminal_state(current_state)) {
                break;
            }
            was_visited_before = (++(in_arc_it_t(graph,current_node)))!=INVALID;
        }
        DEBUG_OUT(2,"...reached leaf-node");
        leaf_node = current_node;
        leaf_state = current_state;
    }

    /* =======================================
       get a heuristic value estimate
       ======================================= */
    (*value_heuristic)(leaf_node,
                       leaf_state,
                       discount,
                       environment,
                       mcts_node_info_map);

    /* =======================================
       backpropagate
       ======================================= */

    DEBUG_OUT(2,"Backup nodes...");

    /* We need to propagate back the returns to allow MC backups. If backup_type
     * is BACKUP_TRACE we also do the backups here. In that case only nodes that
     * lie on the trajectory will be backed up. If graph_type is TREE this
     * cannot be done differently but in a DAG we could backup more nodes (see
     * BACKUP_ALL). */
    {
        // initialize discounted return of this rollout with leaf-node's return
        reward_t discounted_return = mcts_node_info_map[leaf_node].get_return_sum()/mcts_node_info_map[leaf_node].get_rollout_counts();
        // follow the trace back to root node
        node_t observation_node, action_node;
        arc_t to_action_arc, to_observation_arc;
        reward_t reward;
        for(auto transition=trajectory.rbegin(); transition!=trajectory.rend(); ++transition) {
            t(observation_node,to_action_arc,action_node,to_observation_arc,reward) = *transition;
            discounted_return = reward + discount*discounted_return;
            // update counts, reward, and return
            mcts_node_info_map[observation_node  ].add_rollout_on_trajectory(discounted_return);
            mcts_node_info_map[action_node       ].add_rollout_on_trajectory(discounted_return);
            mcts_arc_info_map[to_action_arc      ].add_transition(reward);
            mcts_arc_info_map[to_observation_arc ].add_transition(reward);
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
                (*backup_method)(observation_node,
                                 action_node,
                                 discount,
                                 environment,
                                 graph,
                                 node_info_map,
                                 mcts_node_info_map,
                                 mcts_arc_info_map);
            }
        }
    }

    if(backup_type==BACKUP_TRACE) {
        // backups were done above
    } else if(backup_type==BACKUP_ALL) {
        /* Backups will be performed for all nodes thay lie on a path from the
         * root node to the current node. For graph_type TREE this is identical
         * to BACKUP_TRACE but for DAGs it is not. */

        // hash sets that contain all observation nodes that changed so that their
        // ancestors need to be backed up
        shared_ptr<node_set_t> currently_active_observation_nodes(new node_set_t(0,node_hash));
        shared_ptr<node_set_t> next_active_observation_nodes(new node_set_t({leaf_node},0,node_hash));

        // process nodes
        while(!next_active_observation_nodes->empty()) {
            // swap sets
            currently_active_observation_nodes.swap(next_active_observation_nodes);
            next_active_observation_nodes->clear();
            for(node_t current_node : *currently_active_observation_nodes) {
                // iterate through all observation-action pairs that can lead to this
                // observation node
                for(in_arc_it_t to_observation_arc(graph,current_node);
                    to_observation_arc!=INVALID;
                    ++to_observation_arc) {
                    node_t action_node = graph.source(to_observation_arc);
                    arc_t to_action_arc = in_arc_it_t(graph,action_node);
                    node_t observation_node = graph.source(to_action_arc);
                    (*backup_method)(observation_node,
                                     action_node,
                                     discount,
                                     environment,
                                     graph,
                                     node_info_map,
                                     mcts_node_info_map,
                                     mcts_arc_info_map);
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
                    // add observation node to queue
                    next_active_observation_nodes->insert(observation_node);
                }
            }
        }
    } else DEBUG_DEAD_LINE;
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
            min_val = std::min(min_val, mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_counts());
            max_val = std::max(max_val, mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_counts());
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
        double value = mcts_arc_info_map[arc].get_reward_sum()/mcts_arc_info_map[arc].get_counts();
        if(node_info_map[source].type==OBSERVATION_NODE) {
            arc_map[arc] = QString("style=dashed label=<#%1<BR/>r=%4> color=\"%2 %3 %3\"").
                arg(mcts_arc_info_map[arc].get_counts()).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm))).
                arg(mcts_arc_info_map[arc].get_reward_sum());
        } else {
            arc_map[arc] = QString("style=solid label=<#%2<BR/>r=%6> penwidth=%3 color=\"%4 %5 %5\"").
                arg(mcts_arc_info_map[arc].get_counts()).
                arg(5.*mcts_arc_info_map[arc].get_counts()/mcts_node_info_map[source].get_transition_counts()+0.1).
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

double MonteCarloTreeSearch::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}
