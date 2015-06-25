#ifndef MONTECARLOTREESEARCH_H_
#define MONTECARLOTREESEARCH_H_

#include <memory>
#include <vector>
#include <deque>
#include <list>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <limits>

#include "SearchTree.h"

#include "../graph_util.h"

namespace tree_policy{class TreePolicy;}
namespace value_heuristic{class ValueHeuristic;}
namespace backup_method{class BackupMethod;}

class MonteCarloTreeSearch: public SearchTree {

    //----typedefs/classes----//
public:
    typedef AbstractEnvironment::action_container_t action_container_t;
    struct RolloutItem {
        RolloutItem(action_handle_t action = nullptr,
                    observation_handle_t observation = nullptr,
                    reward_t reward = 0,
                    reward_t discounted_return = 0,
                    double weight = 1,
                    std::shared_ptr<RolloutItem> next = nullptr,
                    bool is_new = true,
                    NODE_TYPE type = OBSERVATION_NODE):
            action(action), observation(observation), reward(reward),
            discounted_return(discounted_return),
            weight(weight), next(next), is_new(is_new), type(type)
        {}
        action_handle_t action;
        observation_handle_t observation;
        reward_t reward;
        reward_t discounted_return;
        double weight;
        std::shared_ptr<RolloutItem> next;
        bool is_new;
        NODE_TYPE type;
    };
    typedef std::set<std::shared_ptr<RolloutItem>> rollout_set_t;
    /**
     * Node-specific data for MCTS.*/
    struct MCTSNodeInfo {
        void set_value(reward_t val,
                       reward_t val_variance,
                       reward_t min_val,
                       reward_t max_val);
        /** # of actions taken from here. */
        int action_counts = 0;
        /** # of added rollout returns. */
        int rollout_counts = 0;
        /** Value of this action/state. */
        reward_t value = 0;
        /** Variance of the value. */
        reward_t value_variance = 0;
        /** Lowest possible value. */
        reward_t min_value = std::numeric_limits<reward_t>::max();
        /** Highest possible value. */
        reward_t max_value = std::numeric_limits<reward_t>::lowest();
        /** Sum of returns for all rollouts. */
        reward_t return_sum = 0;
        /** Sum of squared returns for all rollouts. */
        reward_t squared_return_sum = 0;
        /** Lowest return ever encountered on a rollout. */
        reward_t min_return = std::numeric_limits<reward_t>::max();
        /** Highest return ever encountered on a rollout. */
        reward_t max_return = std::numeric_limits<reward_t>::lowest();
        /** List of all rollouts in leaf nodes. */
        rollout_set_t rollout_set;
    };
    typedef graph_t::NodeMap<MCTSNodeInfo> mcts_node_info_map_t;

    /**
     * Arc-specific data for MCTS.*/
    struct MCTSArcInfo {
        /** Number of times this arc was taken. */
        int transition_counts = 0;
        /** Sum of rewards for all transitions. */
        reward_t reward_sum = 0;
        /** Sum of squared rewards for all transitions. */
        reward_t squared_reward_sum = 0;
        /** Lowest reward ever encountered. */
        reward_t min_reward = std::numeric_limits<reward_t>::max();
        /** Highest reward ever encountered. */
        reward_t max_reward = std::numeric_limits<reward_t>::lowest();
    };
    typedef graph_t::ArcMap<MCTSArcInfo>   mcts_arc_info_map_t;
    typedef std::vector<std::tuple<action_handle_t,double>> action_value_list_t;
protected:
    typedef graph_util::NodeHashFunction<graph_t> node_hash_function_t;
    typedef std::unordered_set<node_t,node_hash_function_t> node_set_t;
public:
    enum class BACKUP_TYPE { TRACE, PROPAGATE };
    enum class ROLLOUT_STORAGE { NONE, CONDENSED, FULL };

    //----members----//
public:
    ROLLOUT_STORAGE rollout_storage;
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

    const BACKUP_TYPE backup_type = BACKUP_TYPE::TRACE;

    const node_hash_function_t node_hash;

    /**
     * Policy that is being used to recommend actions from the root node. */
    std::shared_ptr<tree_policy::TreePolicy> recommendation_policy;

    int max_depth;
    int rollout_length;

    //----methods----//
public:
    MonteCarloTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                         double discount,
                         std::shared_ptr<node_finder::NodeFinder> node_finder,
                         std::shared_ptr<tree_policy::TreePolicy> tree_policy,
                         std::shared_ptr<value_heuristic::ValueHeuristic> value_heuristic,
                         std::shared_ptr<backup_method::BackupMethod> backup_method,
                         BACKUP_TYPE backup_type = BACKUP_TYPE::PROPAGATE,
                         int rollout_length = -1,
                         std::shared_ptr<tree_policy::TreePolicy> recommendation_policy = nullptr,
                         int max_depth = -1,
                         ROLLOUT_STORAGE rollout_storage = ROLLOUT_STORAGE::NONE);
    virtual ~MonteCarloTreeSearch() = default;
    virtual void next_do() override;
    virtual action_handle_t recommend_action() const override;
    virtual action_value_list_t get_action_values() const;
    void plot_graph(const char* file_name,
                    const char* command = "dot",
                    const char* parameters = "-Tpdf",
                    bool delete_dot_file = true) const override;
    int get_max_depth() const {return max_depth;}
    void set_max_depth(int depth) {max_depth = depth;}
    virtual const mcts_node_info_map_t & get_mcts_node_info_map() const;
    virtual const mcts_arc_info_map_t & get_mcts_arc_info_map() const;
protected:
    virtual arc_node_t find_or_create_observation_node(const node_t & action_node,
                                                       const observation_handle_t & observation) override;
    virtual bool transfer_rollouts(node_t from_observation_node,
                                   arc_t to_action_arc,
                                   node_t via_action_node,
                                   arc_t to_observation_arc,
                                   node_t to_observation_node);
    virtual void add_transition(node_t from_observation_node,
                                arc_t to_action_arc,
                                node_t action_node,
                                arc_t to_observation_arc,
                                reward_t reward);
    virtual void add_rollout(node_t node,
                             std::shared_ptr<RolloutItem> rollout);
    virtual std::shared_ptr<RolloutItem> rollout(node_t leaf_node);
    virtual double color_rescale(const double&) const;
    static bool equal(const MCTSNodeInfo & lhs, const MCTSNodeInfo & rhs);
    static void update(const MCTSNodeInfo & from, MCTSNodeInfo & to);
};

#endif /* MONTECARLOTREESEARCH_H_ */
