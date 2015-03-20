#ifndef VALUEHEURISTIC_H_
#define VALUEHEURISTIC_H_

#include "AbstractMonteCarloTreeSearch.h"

class Environment;

namespace value_heuristic {


    typedef AbstractMonteCarloTreeSearch::graph_t              graph_t;
    typedef AbstractMonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_t               node_t;
    typedef AbstractMonteCarloTreeSearch::action_t             action_t;

    /**
     * Abstract basis class for heuristics that initialize the return of new
     * leaf nodes. For problems with a single reward at a terminal state (like
     * the games of chess or go) this typically is a CompleteRollout. In the
     * case of intermediate reward this may also be a PartialRollout of fixed
     * length \e k or OneStep rollout. */
    class ValueHeuristic {
    public:
        virtual void operator()(const node_t & state_node,
                                const Environment & environment,
                                mcts_node_info_map_t & mcts_node_info_map) const = 0;
    };

    /**
     * This heuristic initializes the value to zero. */
    class Zero: public ValueHeuristic {
    public:
        virtual void operator()(const node_t & state_node,
                                const Environment & environment,
                                mcts_node_info_map_t & mcts_node_info_map) const override;
    };

} // end namespace value_heuristic

#endif /* VALUEHEURISTIC_H_ */
