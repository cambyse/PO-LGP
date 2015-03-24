#ifndef MONTECARLOTREESEARCH_H_
#define MONTECARLOTREESEARCH_H_

#include <memory>

#include "AbstractMonteCarloTreeSearch.h"
#include "TreePolicy.h"
#include "ValueHeuristic.h"
#include "BackupMethod.h"

class MonteCarloTreeSearch: public AbstractMonteCarloTreeSearch {

    //----typedefs/classes----//
public:
    enum BACKUP_TYPE { BACKUP_TRACE, BACKUP_ALL };

    //----members----//
protected:
    /**
     * The tree policy that is being used. */
    std::shared_ptr<const tree_policy::TreePolicy> tree_policy;
    /**
     * The value heuristic that is being used. */
    std::shared_ptr<const value_heuristic::ValueHeuristic> value_heuristic;
    /**
     * The backup method that is being used. */
    std::shared_ptr<const backup_method::BackupMethod> backup_method;

    const BACKUP_TYPE backup_type;

    //----methods----//
public:
    MonteCarloTreeSearch(const state_t & root_state,
                         std::shared_ptr<const Environment> environment,
                         double discount,
                         GRAPH_TYPE graph_type,
                         std::shared_ptr<const tree_policy::TreePolicy> tree_policy,
                         std::shared_ptr<const value_heuristic::ValueHeuristic> value_heuristic,
                         std::shared_ptr<const backup_method::BackupMethod> backup_method,
                         BACKUP_TYPE backup_type = BACKUP_ALL);
    virtual ~MonteCarloTreeSearch() = default;
    void next() override;
    action_t recommend_action() const override;
};

#endif /* MONTECARLOTREESEARCH_H_ */
