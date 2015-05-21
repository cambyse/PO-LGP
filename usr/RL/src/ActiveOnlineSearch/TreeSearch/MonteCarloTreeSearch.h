#ifndef MONTECARLOTREESEARCH_H_
#define MONTECARLOTREESEARCH_H_

#include <memory>
#include <unordered_set>
#include <unordered_map>

#include "SearchTree.h"

#include "../graph_util.h"

namespace tree_policy{class TreePolicy;}
namespace value_heuristic{class ValueHeuristic;}
namespace backup_method{class BackupMethod;}

class MonteCarloTreeSearch: public SearchTree {

    //----typedefs/classes----//
public:
    typedef AbstractEnvironment::action_container_t action_container_t;
    /**
     * Node-specific data for MCTS.*/
    class MCTSNodeInfo {
    public:
        MCTSNodeInfo() = default;
        ~MCTSNodeInfo() = default;
        int get_transition_counts() const;
        int get_rollout_counts() const;
        reward_t get_value() const;
        reward_t get_value_variance() const;
        reward_t get_return_sum() const;
        reward_t get_squared_return_sum() const;
        void set_value(reward_t val, reward_t val_variance);
        void add_rollout_return(reward_t ret);
        void add_transition();
        state_handle_t get_state_from_last_visit() const;
        void set_state_from_last_visit(state_handle_t state);
    protected:
        /**
         * Nr of times a transition from this node (state or action) was
         * sampled. */
        int transition_counts = 0;
        /**
         * Nr of times a rollout passed through this node (state or
         * action). */
        int rollout_counts = 0;
        /**
         * Value of this action/state. */
        reward_t value = 0;
        /**
         * Variance of the value. */
        reward_t value_variance = 0;
        /**
         * The sum of returns for all rollouts passing through this
         * node. Monte-Carlo backups compute the \e value as \e
         * return_sum/counts.*/
        reward_t return_sum = 0;
        /**
         * The sum of squared returns for all rollouts passing through this
         * node. */
        reward_t squared_return_sum = 0;
        /**
         * Holds a state handle to the state from the last time the node was
         * visited. */
        state_handle_t state_from_last_visit;
    };
    typedef graph_t::NodeMap<MCTSNodeInfo> mcts_node_info_map_t;

    /**
     * Arc-specific data for MCTS.*/
    class MCTSArcInfo {
    public:
        MCTSArcInfo() = default;
        ~MCTSArcInfo() = default;
        int get_transition_counts() const;
        int get_rollout_counts() const;
        reward_t get_reward_sum() const;
        reward_t get_squared_reward_sum() const;
        void add_rollout_return(reward_t reward);
        void add_transition();
    protected:
        /**
         * Number of times this arc was taken on a transition rollout. For
         * action-->state arcs this corresponds to the (unnormalized) transition
         * probability. For state-->action arcs this correpsonds to the
         * (unnormalized) policy. */
        int transition_counts = 0;
        /**
         * Nr of times a rollout passed through this arc. */
        int rollout_counts = 0;
        /**
         * Sum of rewards of all transitions taking this arc. For action-->state
         * arcs this corresponds to the (unnormalized) expected reward as a
         * function of action and target-state. For state-->action arcs this
         * corresponds to the (unnormalized) expected reward as a function of
         * source-state and action. */
        reward_t reward_sum = 0;
        /**
         * Sum of squared rewards of all transitions taking this arc. */
        reward_t squared_reward_sum = 0;
    };
    typedef graph_t::ArcMap<MCTSArcInfo>   mcts_arc_info_map_t;
protected:
    typedef graph_util::NodeHashFunction<graph_t> node_hash_function_t;
    typedef std::unordered_set<node_t,node_hash_function_t> node_set_t;
public:
    enum BACKUP_TYPE { BACKUP_TRACE, BACKUP_ALL};

    //----members----//
protected:
    /**
     * Map holding node-specific information of type MCTSNodeInfo . */
    mcts_node_info_map_t mcts_node_info_map;
    /**
     * Map holding arc-specific information of type MCTSArcInfo. */
    mcts_arc_info_map_t mcts_arc_info_map;
    /**
     * The tree policy that is being used. */
    std::shared_ptr<tree_policy::TreePolicy> tree_policy;
    /**
     * The value heuristic that is being used. */
    std::shared_ptr<value_heuristic::ValueHeuristic> value_heuristic;
    /**
     * The backup method that is being used. */
    std::shared_ptr<backup_method::BackupMethod> backup_method;

    const BACKUP_TYPE backup_type = BACKUP_TRACE;

    const node_hash_function_t node_hash;

    int max_depth;

    //----methods----//
public:
    MonteCarloTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                         double discount,
                         std::shared_ptr<NodeFinder> node_finder,
                         std::shared_ptr<tree_policy::TreePolicy> tree_policy,
                         std::shared_ptr<value_heuristic::ValueHeuristic> value_heuristic,
                         std::shared_ptr<backup_method::BackupMethod> backup_method,
                         BACKUP_TYPE backup_type = BACKUP_ALL,
                         int max_depth = -1);
    virtual ~MonteCarloTreeSearch() = default;
    virtual void next_do() override;
    virtual action_handle_t recommend_action() const override;
    void toPdf(const char* file_name) const override;
    int get_max_depth() const {return max_depth;}
    void set_max_depth(int depth) {max_depth = depth;}
protected:
    virtual double color_rescale(const double&) const;
};

#endif /* MONTECARLOTREESEARCH_H_ */
