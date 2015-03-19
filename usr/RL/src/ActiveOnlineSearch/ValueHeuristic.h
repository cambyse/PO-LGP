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
        //----typedefs/classes----//

        //----members----//
    protected:
        std::shared_ptr<const Environment> environment;
        mcts_node_info_map_t & mcts_node_info_map;
        //----methods----//
    public:
        ValueHeuristic(std::shared_ptr<const Environment> environment,
                       mcts_node_info_map_t & mcts_node_info_map);
        ~ValueHeuristic() = default;
        virtual void get_value(const node_t &) const = 0;
    };

    /**
     * This heuristic initializes the value to zero. */
    class Zero: public ValueHeuristic {
    public:
        Zero(std::shared_ptr<const Environment> environment,
             mcts_node_info_map_t & mcts_node_info_map);
        ~Zero() = default;
        virtual void get_value(const node_t &) const override;
    };

} // end namespace value_heuristic

#endif /* VALUEHEURISTIC_H_ */
