#ifndef VALUEHEURISTIC_H_
#define VALUEHEURISTIC_H_

#include "MonteCarloTreeSearch.h"

namespace value_heuristic {

    typedef MonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef MonteCarloTreeSearch::node_t               node_t;
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
        friend std::ostream& operator<<(std::ostream & out, const ValueHeuristic & heuristic) {
            heuristic.write(out);
            return out;
        }
        virtual void write(std::ostream &) const = 0;
    };

    /**
     * This heuristic uses zero to initialize value/return. */
    class Zero: public ValueHeuristic {
    public:
        virtual ~Zero() = default;
        virtual void add_value_estimate(const node_t & state_node,
                                        mcts_node_info_map_t & mcts_node_info_map) override;
        virtual void write(std::ostream & out) const override{out<<"Zero()";}
    };

    /**
     * This heuristic does a rollout to initialize value/return. */
    class RolloutStatistics: public ValueHeuristic {
    public:
        /**
         * Constructor with rollout length. For negative values the rollout is
         * either one step (if the environment does not have a terminal state)
         * or infinite until reaching a terminal state. */
        RolloutStatistics(double prior_counts = -1);
        virtual ~RolloutStatistics() = default;
        virtual void init(double disc,
                          std::shared_ptr<AbstractEnvironment> env);
        virtual void add_value_estimate(const node_t & state_node,
                                        mcts_node_info_map_t & mcts_node_info_map) override;
        virtual RolloutStatistics & prior_counts(double prior_counts);
        virtual void write(std::ostream & out) const override{out<<"RolloutStatistics(discount="<<discount<<")";}
    protected:
        double _prior_counts;
        /** If false the variance is computed as the mean squared deviation from
         * the mean value (i.e. the sample variance), if true this term is then
         * corrected by multiplying it with n/(n-1) with n being the number of
         * samples, so that it become an estimate of the true (distribution)
         * variance. */
        static const bool remove_bias = false;
    };

} // end namespace value_heuristic

#endif /* VALUEHEURISTIC_H_ */
