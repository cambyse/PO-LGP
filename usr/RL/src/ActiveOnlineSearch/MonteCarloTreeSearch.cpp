#include "MonteCarloTreeSearch.h"

#include <vector>

#include <util/util.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <util/return_tuple.h>
#include <util/return_tuple_macros.h>

using lemon::INVALID;
using tree_policy::TreePolicy;
using value_heuristic::ValueHeuristic;
using backup_method::BackupMethod;

MonteCarloTreeSearch::MonteCarloTreeSearch(const state_t & root_state,
                                           Environment & environment,
                                           double discount,
                                           const TreePolicy & tree_policy,
                                           const ValueHeuristic & value_heuristic,
                                           const BackupMethod & backup_method):
    AbstractMonteCarloTreeSearch(root_state, environment, discount),
    tree_policy(tree_policy),
    value_heuristic(value_heuristic),
    backup_method(backup_method)
    /* tree_policy(new T(environment, graph, node_info_map, mcts_node_info_map)), */
    /* value_heuristic(new V(environment, mcts_node_info_map)), */
    /* backup_method(new B(discount, environment, graph, mcts_node_info_map, mcts_arc_info_map)) */
{}


void MonteCarloTreeSearch::next() {

    node_t current_node = root_node;

    /* ========================================================================
       (1) follow tree-policy to leaf-node (2) go one more step to expand this
       leaf node (3) stop if newly created node is a leaf or node or goto 1 if
       it is a non-leaf node (only in DAGs)
       ======================================================================== */
    DEBUG_OUT(2,"Follow tree-policy...");
    bool did_expansion = false;
    bool is_inner_node = is_fully_expanded(current_node);
    while(is_inner_node || !did_expansion) {

        /* // expand when being in a leaf-node */
        /* if(is_leaf(current_node)) { */
        /*     DEBUG_OUT(2,"    expanding leaf-node"); */
        /*     expand_leaf(current_node); */
        /*     reached_leaf_node = true; */
        /* } else { */
        /*     // in case of DAGs expanding a leaf-node may create a child that is */
        /*     // not a leaf-node anymore */
        /*     if(reached_leaf_node) { */
        /*         reached_leaf_node = false; */
        /*         DEBUG_OUT(2,"    new leaf-node rejoined DAG"); */
        /*     } */
        /* } */

        // get tree-policy action
        action_t action = tree_policy(current_node,
                                      environment,
                                      graph,
                                      node_info_map,
                                      mcts_node_info_map,
                                      mcts_arc_info_map);

        // find action node
        T(arc_t, to_action_arc, node_t, action_node) = find_action_node(current_node, action);

        // sample state
        T(state_t, state_to, reward_t, reward) = environment.sample(node_info_map[current_node].state, action);

        // find state node / update current state
        T(arc_t, to_state_arc, node_t, state_node) = find_state_node(action_node, state_to);

        // update maps
        mcts_node_info_map[current_node].counts += 1;
        mcts_arc_info_map[to_action_arc].counts += 1;
        mcts_node_info_map[action_node ].counts += 1;
        mcts_arc_info_map[to_state_arc ].counts += 1;
        mcts_arc_info_map[to_action_arc].reward_sum += reward;
        mcts_arc_info_map[to_state_arc ].reward_sum += reward;

        // update node
        mcts_node_info_map[state_node].backtrace = std::make_tuple(current_node,
                                                                   to_action_arc,
                                                                   action_node,
                                                                   to_state_arc);
        current_node = state_node;

        // update halting conditions (last step was an expansion if the node was
        // not an inner node)
        did_expansion = !is_inner_node;
        is_inner_node = is_fully_expanded(current_node);
    }
    DEBUG_OUT(2,"...reached leaf-node");

    /* =======================================
       get a heuristic value estimate
       ======================================= */
    value_heuristic(current_node, environment, mcts_node_info_map);

    /* =======================================
       backpropagate
       ======================================= */
    DEBUG_OUT(2,"Backup nodes...");
    node_t action_node;
    arc_t to_action_arc, to_state_arc;
    auto trace = t(current_node, to_action_arc, action_node, to_state_arc);
    for(trace = mcts_node_info_map[current_node].backtrace;
        current_node!=INVALID;
        trace = mcts_node_info_map[current_node].backtrace) {
        DEBUG_OUT(2,"    backup node (" << graph.id(current_node) <<
                  "): value = " << mcts_node_info_map[current_node].value);
        backup_method(current_node,
                      action_node,
                      discount,
                      environment,
                      graph,
                      mcts_node_info_map,
                      mcts_arc_info_map);
    }
}


Environment::action_t MonteCarloTreeSearch::recommend_action() const {
    std::vector<action_t> optimal_actions({*(environment.actions.begin())});
    double max_value = -DBL_MAX;
    for(out_arc_it_t arc(graph, root_node); arc!=lemon::INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        double value = mcts_node_info_map[action_node].value;
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
