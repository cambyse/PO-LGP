#ifndef MONTECARLOTREESEARCH_H_
#define MONTECARLOTREESEARCH_H_

#include <vector>

#include <util/util.h>

#include "AbstractMonteCarloTreeSearch.h"

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
        value_heuristic(new V()),
        backup_method(new B())
    {}

template<class T, class V, class B>
        void MonteCarloTreeSearch<T,V,B>::next() {
    node_t current_node = root_node;
    while(!is_leaf(current_node)) {
        current_node = tree_policy->next(current_node);
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


#endif /* MONTECARLOTREESEARCH_H_ */
