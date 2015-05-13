#ifndef ABSTRACTMONTECARLOTREESEARCH_H_
#define ABSTRACTMONTECARLOTREESEARCH_H_

#include "SearchTree.h"

class AbstractMonteCarloTreeSearch: public SearchTree {

    //----typedefs/classes----//

public:

    /**
     * Node-specific data for MCTS.*/
    class MCTSNodeInfo {
    public:
        MCTSNodeInfo() = default;
        ~MCTSNodeInfo() = default;
        int get_transition_counts() const {return transition_counts;}
        int get_rollout_counts() const {return rollout_counts;}
        reward_t get_value() const {return value;}
        reward_t get_value_variance() const {return value_variance;}
        reward_t get_return_sum() const {return return_sum;}
        reward_t get_squared_return_sum() const {return squared_return_sum;}
        void set_value(reward_t val, reward_t val_variance) {
            value=val;
            value_variance=val_variance;
        }
        void add_separate_rollout(reward_t ret) {
            ++rollout_counts;
            return_sum+=ret;
            squared_return_sum+=ret*ret;
        }
        void add_rollout_on_trajectory(reward_t ret) {
            add_separate_rollout(ret);
            ++transition_counts;
        }
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
    };
    typedef graph_t::NodeMap<MCTSNodeInfo> mcts_node_info_map_t;

    /**
     * Arc-specific data for MCTS.*/
    class MCTSArcInfo {
    public:
        MCTSArcInfo() = default;
        ~MCTSArcInfo() = default;
        int get_counts() const {return counts;}
        reward_t get_reward_sum() const {return reward_sum;}
        reward_t get_squared_reward_sum() const {return squared_reward_sum;}
        void add_transition(reward_t reward) {
            ++counts;
            reward_sum+=reward;
            squared_reward_sum+=reward*reward;
        }
    protected:
        /**
         * Number of times this arc was taken on a rollout. For
         * action-->state arcs this corresponds to the (unnormalized) transition
         * probability. For state-->action arcs this correpsonds to the
         * (unnormalized) policy. */
        int counts = 0;
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

    //----members----//
protected:
    /**
     * Const reference to graph of SearchTree basis class. */
    const graph_t & graph;
    /**
     * Map holding node-specific information of type MCTSNodeInfo . */
    mcts_node_info_map_t mcts_node_info_map;
    /**
     * Map holding arc-specific information of type MCTSArcInfo. */
    mcts_arc_info_map_t mcts_arc_info_map;

    //----methods----//
public:
    AbstractMonteCarloTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                                 double discount,
                                 GRAPH_TYPE graph_type);
    virtual ~AbstractMonteCarloTreeSearch() = default;
    void toPdf(const char* file_name) const override;
};

#endif /* ABSTRACTMONTECARLOTREESEARCH_H_ */
