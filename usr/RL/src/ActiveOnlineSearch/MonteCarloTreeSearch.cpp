#include "MonteCarloTreeSearch.h"

#include <vector>
#include <tuple>
#include <unordered_set>
#include <functional>

#include <util/util.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <util/return_tuple.h>
#include <util/return_tuple_macros.h>

using lemon::INVALID;
using tree_policy::TreePolicy;
using value_heuristic::ValueHeuristic;
using backup_method::BackupMethod;
using std::unordered_set;
using std::tuple;
using std::vector;

MonteCarloTreeSearch::MonteCarloTreeSearch(const state_t & root_state,
                                           std::shared_ptr<const Environment> environment,
                                           double discount,
                                           GRAPH_TYPE graph_type,
                                           std::shared_ptr<const TreePolicy> tree_policy,
                                           std::shared_ptr<const ValueHeuristic> value_heuristic,
                                           std::shared_ptr<const BackupMethod> backup_method,
                                           BACKUP_TYPE backup_type):
    AbstractMonteCarloTreeSearch(root_state, environment, discount, graph_type),
    tree_policy(tree_policy),
    value_heuristic(value_heuristic),
    backup_method(backup_method),
    backup_type(backup_type)
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
    {
        DEBUG_OUT(2,"Follow tree-policy...");
        node_t current_node = root();
        bool did_expansion = false;
        bool is_inner_node = is_fully_expanded(current_node);
        bool was_visited_before = false;
        while(is_inner_node || !did_expansion || was_visited_before) {

            // get tree-policy action
            action_t action = (*tree_policy)(current_node,
                                             environment,
                                             graph,
                                             get_node_info_map(),
                                             mcts_node_info_map,
                                             mcts_arc_info_map);

            // find or create action node
            T(arc_t, to_action_arc, node_t, action_node) = find_or_create_action_node(current_node, action);

            // sample state
            T(state_t, state_to, reward_t, reward) = environment->sample(state(current_node), action);

            // find or create state node / update current state
            T(arc_t, to_state_arc, node_t, state_node) = find_or_create_state_node(action_node, state_to);

            // add to trajectory
            trajectory.push_back(trajectory_item_t(current_node, to_action_arc, action_node, to_state_arc, reward));

            // update node
            current_node = state_node;

            // update halting conditions (last step was an expansion if the node was
            // not an inner node)
            did_expansion = !is_inner_node;
            is_inner_node = is_fully_expanded(current_node);
            if(environment->is_terminal_state(state_to)) {
                break;
            }
            was_visited_before = (++(in_arc_it_t(graph,current_node)))!=INVALID;
        }
        DEBUG_OUT(2,"...reached leaf-node");
        leaf_node = current_node;
    }

    /* =======================================
       get a heuristic value estimate
       ======================================= */
    (*value_heuristic)(leaf_node,
                       state(leaf_node),
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
        node_t state_node, action_node;
        arc_t to_action_arc, to_state_arc;
        reward_t reward;
        for(auto transition=trajectory.rbegin(); transition!=trajectory.rend(); ++transition) {
            t(state_node,to_action_arc,action_node,to_state_arc,reward) = *transition;
            discounted_return = reward + discount*discounted_return;
            // update counts, reward, and return
            mcts_node_info_map[state_node  ].add_rollout_on_trajectory(discounted_return);
            mcts_node_info_map[action_node ].add_rollout_on_trajectory(discounted_return);
            mcts_arc_info_map[to_action_arc].add_transition(reward);
            mcts_arc_info_map[to_state_arc ].add_transition(reward);
            DEBUG_OUT(2,QString("    update state-node(%1):	counts=%2/%3	return_sum=%4").
                      arg(graph.id(state_node)).
                      arg(mcts_node_info_map[state_node].get_transition_counts()).
                      arg(mcts_node_info_map[state_node].get_rollout_counts()).
                      arg(mcts_node_info_map[state_node].get_return_sum()));
            DEBUG_OUT(2,QString("    update action-node(%1):	counts=%2/%3	return_sum=%4").
                      arg(graph.id(action_node)).
                      arg(mcts_node_info_map[action_node].get_transition_counts()).
                      arg(mcts_node_info_map[action_node].get_rollout_counts()).
                      arg(mcts_node_info_map[action_node].get_return_sum()));
            DEBUG_OUT(2,QString("    reward=%1, return=%2").arg(reward).arg(discounted_return));
            // backup if back_type is BACKUP_TRACE
            if(backup_type==BACKUP_TRACE) {
                (*backup_method)(state_node,
                                 action_node,
                                 discount,
                                 environment,
                                 graph,
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

        // hash sets that contain all state nodes that changed so that their
        // ancestors need to be backed up
        typedef unordered_set<node_t,std::function<int(node_t)>> node_set_t;
        node_set_t currently_active_state_nodes(0,[&](node_t n){return graph.id(n);});
        node_set_t next_active_state_nodes({leaf_node},1,[&](node_t n){return graph.id(n);});

        // process nodes
        while(!next_active_state_nodes.empty()) {
            // swap sets
            currently_active_state_nodes.swap(next_active_state_nodes);
            next_active_state_nodes.clear();
            for(node_t current_node : currently_active_state_nodes) {
                // iterate through all state-action pairs that can lead to this
                // state node
                for(in_arc_it_t to_state_arc(graph,current_node);
                    to_state_arc!=INVALID;
                    ++to_state_arc) {
                    node_t action_node = graph.source(to_state_arc);
                    arc_t to_action_arc = in_arc_it_t(graph,action_node);
                    node_t state_node = graph.source(to_action_arc);
                    (*backup_method)(state_node,
                                     action_node,
                                     discount,
                                     environment,
                                     graph,
                                     mcts_node_info_map,
                                     mcts_arc_info_map);
                    DEBUG_OUT(2,QString("    update state-node(%1):	counts=%2/%3	return_sum=%4").
                              arg(graph.id(state_node)).
                              arg(mcts_node_info_map[state_node].get_transition_counts()).
                              arg(mcts_node_info_map[state_node].get_rollout_counts()).
                              arg(mcts_node_info_map[state_node].get_return_sum()));
                    DEBUG_OUT(2,QString("    update action-node(%1):	counts=%2/%3	return_sum=%4").
                              arg(graph.id(action_node)).
                              arg(mcts_node_info_map[action_node].get_transition_counts()).
                              arg(mcts_node_info_map[action_node].get_rollout_counts()).
                              arg(mcts_node_info_map[action_node].get_return_sum()));
                    // add state node to queue
                    next_active_state_nodes.insert(state_node);
                }
            }
        }
    } else DEBUG_DEAD_LINE;
}


Environment::action_t MonteCarloTreeSearch::recommend_action() const {
    std::vector<action_t> optimal_actions({*(environment->get_actions().begin())});
    double max_value = -DBL_MAX;
    for(out_arc_it_t arc(graph, root()); arc!=lemon::INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        double value = mcts_node_info_map[action_node].get_value();
        if(value>max_value) {
            optimal_actions.clear();
            max_value = value;
        }
        if(value>=max_value) {
            optimal_actions.push_back(action(action_node));
        }
    }
    return util::random_select(optimal_actions);
}
