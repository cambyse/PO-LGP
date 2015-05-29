#ifndef VALUEHEURISTIC_H_
#define VALUEHEURISTIC_H_

#include "AbstractMonteCarloTreeSearch.h"

class Environment;

namespace value_heuristic {


    typedef AbstractMonteCarloTreeSearch::graph_t              graph_t;
    typedef AbstractMonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_t               node_t;
    typedef AbstractEnvironment::action_handle_t               action_handle_t;
    typedef AbstractEnvironment::observation_handle_t          observation_handle_t;
    typedef AbstractEnvironment::state_handle_t                state_handle_t;
    typedef AbstractEnvironment::reward_t                      reward_t;

    /**
     * Abstract basis class for heuristics that compute value and return for new
     * leaf nodes. For problems with a single reward at a terminal state (like
     * the games of chess or go) this typically is a Rollout until the terminal
     * state. In the case of intermediate reward this may also be a Rollout of
     * fixed length \e k just using Zero as initialization. */
    class ValueHeuristic {
    public:
        virtual void operator()(const node_t & state_node,
                                const state_handle_t & state,
                                double discount,
                                std::shared_ptr<const Environment> environment,
                                mcts_node_info_map_t & mcts_node_info_map) const = 0;
    };

    /**
     * This heuristic uses zero to initialize value/return. */
    class Zero: public ValueHeuristic {
    public:
        virtual void operator()(const node_t & state_node,
                                const state_handle_t & state,
                                double discount,
                                std::shared_ptr<const Environment> environment,
                                mcts_node_info_map_t & mcts_node_info_map) const override;
    };

    /**
     * This heuristic does a rollout to initialize value/return. */
    class Rollout: public ValueHeuristic {
    public:
        Rollout(int rollout_length = -1);
        virtual void operator()(const node_t & state_node,
                                const state_handle_t & state,
                                double discount,
                                std::shared_ptr<const Environment> environment,
                                mcts_node_info_map_t & mcts_node_info_map) const override;
    protected:
        int rollout_length;
    };

} // end namespace value_heuristic

#endif /* VALUEHEURISTIC_H_ */
