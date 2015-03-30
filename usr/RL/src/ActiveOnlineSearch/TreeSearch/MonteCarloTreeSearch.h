#ifndef MONTECARLOTREESEARCH_H_
#define MONTECARLOTREESEARCH_H_

#include <memory>
#include <unordered_set>
#include <unordered_map>

#include "AbstractMonteCarloTreeSearch.h"
#include "TreePolicy.h"
#include "ValueHeuristic.h"
#include "BackupMethod.h"

class MonteCarloTreeSearch: public AbstractMonteCarloTreeSearch {

    //----typedefs/classes----//
protected:
    //typedef std::function<int(node_t)> NodeHashFunction;
    /**
     * Computes a hash for nodes. This class returns the node ID given the used
     * graph. It could be a std::function<int(node_t)> initialized with a lambda
     * function like @code [&](node_t n){return graph.id(n);} @endcode. However,
     * std::function has a default constructor while it is important to always
     * have the graph object handed over. Using std::function thus implies a
     * certain risk of misuse if people use the operator::[] on state_node_map
     * without checking if the corresponding node_set_t need to be initialized
     * with a proper NodeHashFunction object (which would result in a
     * std::bad_function_call exception). This is not possible given a separate
     * class without default constructor. */
    struct NodeHashFunction {
    public:
        NodeHashFunction(const graph_t & graph): graph(graph) {}
        int operator()(const node_t & node)const{return graph.id(node);}
    private:
        const graph_t & graph;
    };
    typedef std::unordered_set<node_t,NodeHashFunction> node_set_t;
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
     * Stores all nodes for a specific state. */
    std::unordered_map<state_t,node_set_t> state_node_map;

    const BACKUP_TYPE backup_type;

    const NodeHashFunction node_hash;

    //----methods----//
public:
    MonteCarloTreeSearch(std::shared_ptr<const Environment> environment,
                         double discount,
                         GRAPH_TYPE graph_type,
                         std::shared_ptr<const tree_policy::TreePolicy> tree_policy,
                         std::shared_ptr<const value_heuristic::ValueHeuristic> value_heuristic,
                         std::shared_ptr<const backup_method::BackupMethod> backup_method,
                         BACKUP_TYPE backup_type = BACKUP_ALL);
    virtual ~MonteCarloTreeSearch() = default;
    virtual void init(const state_t & s) override;
    virtual void next() override;
    virtual action_t recommend_action() const override;
    virtual void prune(const action_t &, const state_t &) override;
protected:
    virtual std::tuple<arc_t,node_t> add_state_node(state_t state, node_t action_node) override;
    virtual std::tuple<arc_t,node_t> add_action_node(action_t action, node_t state_node) override;
    virtual void erase_node(node_t) override;
};

#endif /* MONTECARLOTREESEARCH_H_ */
