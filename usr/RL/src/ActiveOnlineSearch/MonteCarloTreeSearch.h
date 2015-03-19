#ifndef MONTECARLOTREESEARCH_H_
#define MONTECARLOTREESEARCH_H_

#include <vector>

#include <util/return_tuple.h>
#include <util/util.h>

#include "AbstractMonteCarloTreeSearch.h"
#include "TreePolicy.h"
#include "ValueHeuristic.h"
#include "BackupMethod.h"

#define DEBUG_LEVEL 4
#include <util/debug.h>

template<class TreePolicy,
    class ValueHeuristic,
    class BackupMethod>
class MonteCarloTreeSearch: public AbstractMonteCarloTreeSearch {

    //----typedefs/classes----//

    friend TreePolicy;
    friend ValueHeuristic;
    friend BackupMethod;

    //----members----//
protected:

    /**
     * The tree policy that is being used. */
    std::shared_ptr<TreePolicy> tree_policy;
    /**
     * The value heuristic that is being used. */
    std::shared_ptr<ValueHeuristic> value_heuristic;
    /**
     * The backup method that is being used. */
    std::shared_ptr<BackupMethod> backup_method;

    //----methods----//
public:
    MonteCarloTreeSearch(const state_t & root_state,
                         std::shared_ptr<Environment> environment,
                         double discount);
    virtual ~MonteCarloTreeSearch() = default;
    void next() override;
    action_t recommend_action() const override;
};

template<class T, class V, class B>
    MonteCarloTreeSearch<T,V,B>::MonteCarloTreeSearch(const state_t & root_state,
                                                      std::shared_ptr<Environment> environment,
                                                      double discount):
    AbstractMonteCarloTreeSearch(root_state, environment, discount),
        tree_policy(new T(environment, graph, node_info_map, mcts_node_info_map)),
        value_heuristic(new V(environment, mcts_node_info_map)),
        backup_method(new B(discount, environment, graph, mcts_node_info_map, mcts_arc_info_map))
    {}

template<class T, class V, class B>
        void MonteCarloTreeSearch<T,V,B>::next() {

#include <util/return_tuple_macros.h>
    using lemon::INVALID;

    node_t current_node = root_node;

    /* ========================================================================
       follow tree-policy to existing leaf-node, expand that leaf-node and go
       one more step to newly created leaf-node
       ======================================================================== */
    DEBUG_OUT(2,"Follow tree-policy...");
    bool newly_expanded = false;
    while(!is_leaf(current_node) || !newly_expanded) {

        // expand when being in a leaf-node
        if(is_leaf(current_node)) {
            DEBUG_OUT(2,"    expanding leaf-node");
            expand_leaf(current_node);
            newly_expanded = true;
        } else {
            // in case of DAGs expanding a leaf-node may create a child that is
            // not a leaf-node anymore
            DEBUG_OUT(2,"    new leaf-node rejoined DAG");
            newly_expanded = false;
        }

        // get tree-policy action
        action_t action = tree_policy->next(current_node);

        // find action node
        T(arc_t, action_arc, node_t, action_node) = find_action_node(current_node, action);

        // sample state
        T(state_t, state_to, reward_t, reward) = environment->sample(node_info_map[current_node].state, action);

        // find state node / update current state
        T(arc_t, state_arc, node_t, state_node) = find_state_node(action_node, state_to);

        // update maps
        mcts_node_info_map[action_node].counts += 1;
        mcts_node_info_map[state_node ].counts += 1;
        mcts_arc_info_map[action_arc  ].counts += 1;
        mcts_arc_info_map[state_arc   ].counts += 1;
        mcts_arc_info_map[action_arc  ].reward_sum += reward;
        mcts_arc_info_map[state_arc   ].reward_sum += reward;

        // update current node
        mcts_node_info_map[state_node].backtrace = std::make_tuple(current_node, action_arc, action_node, state_arc);
        current_node = state_node;
    }
    DEBUG_OUT(2,"...reached leaf-node");

    /* =======================================
            get a heuristic value estimate
       ======================================= */
    value_heuristic->get_value(current_node);

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
        DEBUG_OUT(2,"    backup node (" << graph.id(current_node) << "): value = " << mcts_node_info_map[current_node].value);
        backup_method->backup(current_node, action_node);
    }
}

template<class T, class V, class B>
    Environment::action_t MonteCarloTreeSearch<T,V,B>::recommend_action() const {
    std::vector<action_t> optimal_actions({*(environment->actions.begin())});
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

#include <util/debug_exclude.h>

#endif /* MONTECARLOTREESEARCH_H_ */
