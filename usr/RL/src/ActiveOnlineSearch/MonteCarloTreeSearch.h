#ifndef MONTECARLOTREESEARCH_H_
#define MONTECARLOTREESEARCH_H_

#include "AbstractMonteCarloTreeSearch.h"
#include "TreePolicy.h"
#include "ValueHeuristic.h"
#include "BackupMethod.h"

class MonteCarloTreeSearch: public AbstractMonteCarloTreeSearch {

    //----typedefs/classes----//

    //----members----//
protected:

    /**
     * The tree policy that is being used. */
    const tree_policy::TreePolicy & tree_policy;
    /**
     * The value heuristic that is being used. */
    const value_heuristic::ValueHeuristic & value_heuristic;
    /**
     * The backup method that is being used. */
    const backup_method::BackupMethod & backup_method;

    //----methods----//
public:
    MonteCarloTreeSearch(const state_t & root_state,
                         Environment & environment,
                         double discount,
                         GRAPH_TYPE graph_type,
                         const tree_policy::TreePolicy & tree_policy,
                         const value_heuristic::ValueHeuristic & value_heuristic,
                         const backup_method::BackupMethod & backup_method);
    virtual ~MonteCarloTreeSearch() = default;
    void next() override;
    action_t recommend_action() const override;
};

#endif /* MONTECARLOTREESEARCH_H_ */
