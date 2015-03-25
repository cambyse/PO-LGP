#ifndef TREEPOLICY_H_
#define TREEPOLICY_H_

#include "AbstractMonteCarloTreeSearch.h"

class Environment;

namespace tree_policy {

    typedef AbstractMonteCarloTreeSearch::graph_t              graph_t;
    typedef AbstractMonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef AbstractMonteCarloTreeSearch::mcts_arc_info_map_t  mcts_arc_info_map_t;
    typedef AbstractMonteCarloTreeSearch::node_t               node_t;
    typedef AbstractMonteCarloTreeSearch::arc_t                arc_t;
    typedef AbstractMonteCarloTreeSearch::action_t             action_t;
    typedef AbstractMonteCarloTreeSearch::reward_t             reward_t;

    /**
     * Abstract basis class for tree policies. The job of the tree policy is to
     * traverse the existing search tree until reaching a leaf node. The most
     * common tree policy is UCB1 but a Uniform policy may also be used. */
    class TreePolicy {
    public:
        virtual action_t operator()(const node_t & state_node,
                                    std::shared_ptr<const Environment> environment,
                                    const graph_t & graph,
                                    const node_info_map_t & node_info_map,
                                    const mcts_node_info_map_t & mcts_node_info_map,
                                    const mcts_arc_info_map_t & mcts_arc_info_map) const = 0;
    };

    /**
     * Sample actions uniformly from available action nodes. */
    class Uniform: public TreePolicy {
    public:
        virtual action_t operator()(const node_t & state_node,
                                    std::shared_ptr<const Environment> environment,
                                    const graph_t & graph,
                                    const node_info_map_t & node_info_map,
                                    const mcts_node_info_map_t & mcts_node_info_map,
                                    const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    };

    /**
     * Basis class for policies the sample upper bounds. */
    class UpperBoundPolicy: public TreePolicy {
    public:
        virtual action_t operator()(const node_t & state_node,
                                    std::shared_ptr<const Environment> environment,
                                    const graph_t & graph,
                                    const node_info_map_t & node_info_map,
                                    const mcts_node_info_map_t & mcts_node_info_map,
                                    const mcts_arc_info_map_t & mcts_arc_info_map) const override final;
        virtual reward_t upper_bound(const node_t & state_node,
                                     const arc_t & to_action_arc,
                                     const node_t & action_node,
                                     std::shared_ptr<const Environment> environment,
                                     const graph_t & graph,
                                     const node_info_map_t & node_info_map,
                                     const mcts_node_info_map_t & mcts_node_info_map,
                                     const mcts_arc_info_map_t & mcts_arc_info_map) const = 0;
    };

    /**
     * Sample actions according to UCB1 policy. */
    class UCB1: public UpperBoundPolicy {
    public:
        /**
         * Constructor. @param Cp This is the scaling parameter for
         * exploration. The default value is Cp=1/√2, which was shown by Kocsis
         * and Szepesvári to satisfy the Hoeffding ineqality. */
        UCB1(double Cp = 0.70710678118654746);
        virtual reward_t upper_bound(const node_t & state_node,
                                     const arc_t & to_action_arc,
                                     const node_t & action_node,
                                     std::shared_ptr<const Environment> environment,
                                     const graph_t & graph,
                                     const node_info_map_t & node_info_map,
                                     const mcts_node_info_map_t & mcts_node_info_map,
                                     const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    protected:
        double Cp;
    };

    /**
     * Sample action with maximum upper bound. This is similar to UCB1 except
     * that the bound is computed as \f[ Q_{(s,a)}^+ = \widehat{Q}_{(s,a)} + C_p
     * \sqrt{\frac{\widetilde{Q}_{(s,a)}}{n_{(s,a)}}} \f] where
     * \f$\widehat{Q}_{(s,a)}\f$ is the mean value, \f$\widetilde{Q}_{(s,a)}\f$
     * is the variance of the value, \f$n_{(s,a)}\f$ is the number times action
     * \e a was taken from state \e s and \f$C_p\f$ (as in UCB1) balances
     * exploration and exploitation. These bounds take into account uncertainty
     * further down in the tree. */
    class UCB_Plus: public UpperBoundPolicy {
    public:
        /**
         * Constructor. @param Cp This is the scaling parameter for
         * exploration.*/
        UCB_Plus(double Cp = 1);
        virtual reward_t upper_bound(const node_t & state_node,
                                     const arc_t & to_action_arc,
                                     const node_t & action_node,
                                     std::shared_ptr<const Environment> environment,
                                     const graph_t & graph,
                                     const node_info_map_t & node_info_map,
                                     const mcts_node_info_map_t & mcts_node_info_map,
                                     const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    protected:
        double Cp;
    };

} // end namespace tree_policy

#endif /* TREEPOLICY_H_ */
