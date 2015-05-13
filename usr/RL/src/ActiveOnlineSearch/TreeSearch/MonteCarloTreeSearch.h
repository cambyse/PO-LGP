#ifndef MONTECARLOTREESEARCH_H_
#define MONTECARLOTREESEARCH_H_

#include <memory>
#include <unordered_set>
#include <unordered_map>

#include "AbstractMonteCarloTreeSearch.h"
#include "TreePolicy.h"
#include "ValueHeuristic.h"
#include "BackupMethod.h"

#include "../graph_util.h"

class MonteCarloTreeSearch: public AbstractMonteCarloTreeSearch {

    //----typedefs/classes----//
protected:
    typedef graph_util::NodeHashFunction<graph_t> node_hash_function_t;
    typedef std::unordered_set<node_t,node_hash_function_t> node_set_t;
public:
    enum BACKUP_TYPE { BACKUP_TRACE, BACKUP_ALL, BACKUP_GLOBAL };

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
    /**
     * Distance from the root node. This is needed to determine the order in
     * global backups (backup_type==BACKUP_GLOBAL). */
    graph_t::NodeMap<int> distance_map;
    /**
     * Stores all nodes for a specific observation. */
    std::unordered_map<observation_handle_t,
        node_set_t,
        AbstractEnvironment::hash<observation_handle_t>> observation_node_map;

    const BACKUP_TYPE backup_type;

    const node_hash_function_t node_hash;

    //----methods----//
public:
    MonteCarloTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                         double discount,
                         GRAPH_TYPE graph_type,
                         std::shared_ptr<const tree_policy::TreePolicy> tree_policy,
                         std::shared_ptr<const value_heuristic::ValueHeuristic> value_heuristic,
                         std::shared_ptr<const backup_method::BackupMethod> backup_method,
                         BACKUP_TYPE backup_type = BACKUP_ALL);
    virtual ~MonteCarloTreeSearch() = default;
    virtual void init(const observation_handle_t & o,
                      const state_handle_t & s) override;
    virtual void next() override;
    virtual action_handle_t recommend_action() const override;
    virtual void prune(const action_handle_t &,
                       const observation_handle_t &,
                       const state_handle_t &) override;
protected:
    virtual std::tuple<arc_t,node_t> add_observation_node(observation_handle_t observation,
                                                          node_t action_node) override;
    virtual std::tuple<arc_t,node_t> add_action_node(action_handle_t action,
                                                     node_t observation_node) override;
    virtual void erase_node(node_t) override;
};

#endif /* MONTECARLOTREESEARCH_H_ */
