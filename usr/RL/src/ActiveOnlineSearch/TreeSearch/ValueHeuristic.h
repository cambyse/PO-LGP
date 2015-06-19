#ifndef VALUEHEURISTIC_H_
#define VALUEHEURISTIC_H_

#include "MonteCarloTreeSearch.h"

namespace value_heuristic {

    typedef MonteCarloTreeSearch::graph_t              graph_t;
    typedef MonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef MonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef MonteCarloTreeSearch::node_t               node_t;
    typedef AbstractEnvironment::action_handle_t       action_handle_t;
    typedef AbstractEnvironment::observation_handle_t  observation_handle_t;
    typedef AbstractEnvironment::reward_t              reward_t;

    /**
     * Abstract basis class for heuristics that compute value and return for new
     * leaf nodes. For problems with a single reward at a terminal state (like
     * the games of chess or go) this typically is a Rollout until the terminal
     * state. In the case of intermediate reward this may also be a Rollout of
     * fixed length \e k just using Zero as initialization. */
    class ValueHeuristic {
    public:
        //----members----//
        double discount = 0;
        std::shared_ptr<AbstractEnvironment> environment = nullptr;
    public:
        //----methods----//
        virtual ~ValueHeuristic() = default;
        virtual void init(double discount,
                          std::shared_ptr<AbstractEnvironment> environment);
        virtual void add_value_estimate(const node_t & state_node,
                                        mcts_node_info_map_t & mcts_node_info_map) = 0;
    };

    /**
     * This heuristic uses zero to initialize value/return. */
    class Zero: public ValueHeuristic {
    public:
        virtual ~Zero() = default;
        virtual void add_value_estimate(const node_t & state_node,
                                        mcts_node_info_map_t & mcts_node_info_map) override;
    };

    /**
     * This heuristic does a rollout to initialize value/return. */
    class Rollout: public ValueHeuristic {
    public:
        typedef MonteCarloTreeSearch::ActionObservationReward ActionObservationReward;
        typedef MonteCarloTreeSearch::rollout_t rollout_t;
        /**
         * Constructor with rollout length. For negative values the rollout is
         * either one step (if the environment does not have a terminal state)
         * or infinite until reaching a terminal state. */
        Rollout(int rollout_length = -1, double prior_counts = -1);
        virtual ~Rollout() = default;
        virtual void init(double disc,
                          std::shared_ptr<AbstractEnvironment> env);
        virtual void add_value_estimate(const node_t & state_node,
                                        mcts_node_info_map_t & mcts_node_info_map) override;
        virtual rollout_t get_last_rollout() const {return last_rollout;}
    protected:
        int rollout_length;
        double prior_counts;
        rollout_t last_rollout;
    };

} // end namespace value_heuristic

#endif /* VALUEHEURISTIC_H_ */
