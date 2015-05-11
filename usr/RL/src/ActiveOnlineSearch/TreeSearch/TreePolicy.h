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
    typedef AbstractEnvironment::action_handle_t               action_handle_t;
    typedef AbstractEnvironment::observation_handle_t          observation_handle_t;
    typedef AbstractEnvironment::state_handle_t                state_handle_t;
    typedef AbstractEnvironment::reward_t                      reward_t;

    /**
     * Abstract basis class for tree policies. The job of the tree policy is to
     * traverse the existing search tree until reaching a leaf node. The most
     * common tree policy is UCB1 but a Uniform policy may also be used. */
    class TreePolicy {
    public:
        virtual action_handle_t operator()(const node_t & state_node,
                                           std::shared_ptr<Environment> environment,
                                           const graph_t & graph,
                                           const node_info_map_t & node_info_map,
                                           const mcts_node_info_map_t & mcts_node_info_map,
                                           const mcts_arc_info_map_t & mcts_arc_info_map) const = 0;
    };

    /**
     * Sample actions uniformly from available action nodes. */
    class Uniform: public TreePolicy {
    public:
        virtual action_handle_t operator()(const node_t & state_node,
                                           std::shared_ptr<Environment> environment,
                                           const graph_t & graph,
                                           const node_info_map_t & node_info_map,
                                           const mcts_node_info_map_t & mcts_node_info_map,
                                           const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    };

    /**
     * Basis class for policies that choose the action by maximizing some
     * quantity (like value or upper bound). */
    class MaxPolicy: public TreePolicy {
    public:
        virtual action_handle_t operator()(const node_t & state_node,
                                           std::shared_ptr<Environment> environment,
                                           const graph_t & graph,
                                           const node_info_map_t & node_info_map,
                                           const mcts_node_info_map_t & mcts_node_info_map,
                                           const mcts_arc_info_map_t & mcts_arc_info_map) const override final;
        virtual reward_t score(const node_t & state_node,
                               const arc_t & to_action_arc,
                               const node_t & action_node,
                               std::shared_ptr<Environment> environment,
                               const graph_t & graph,
                               const node_info_map_t & node_info_map,
                               const mcts_node_info_map_t & mcts_node_info_map,
                               const mcts_arc_info_map_t & mcts_arc_info_map) const = 0;
    };

    /**
     * Sample the action with highes value. */
    class Optimal: public MaxPolicy {
    public:
        virtual reward_t score(const node_t & state_node,
                               const arc_t & to_action_arc,
                               const node_t & action_node,
                               std::shared_ptr<Environment> environment,
                               const graph_t & graph,
                               const node_info_map_t & node_info_map,
                               const mcts_node_info_map_t & mcts_node_info_map,
                               const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    };

    /**
     * Sample actions according to UCB1 policy. The upper bound is computed as
     * \f[
     *
     * Q_{(s,a)}^+ = \widehat{Q}_{(s,a)} + 2 C_p \sqrt{2\log n / n_j}
     *
     * \f] where \f$n\f$ and \f$n_j\f$ are the counts of the state node and the arc
     * to the action node, respectively. */
    class UCB1: public MaxPolicy {
    public:
        /**
         * Constructor. @param Cp This is the scaling parameter for
         * exploration. The default value is Cp=1/√2, which was shown by Kocsis
         * and Szepesvári to satisfy the Hoeffding ineqality. */
        UCB1(double Cp = 0.70710678118654746);
        virtual reward_t score(const node_t & state_node,
                               const arc_t & to_action_arc,
                               const node_t & action_node,
                               std::shared_ptr<Environment> environment,
                               const graph_t & graph,
                               const node_info_map_t & node_info_map,
                               const mcts_node_info_map_t & mcts_node_info_map,
                               const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    protected:
        double Cp;
    };

    /**
     * Sample action with maximum upper bound. This is similar to UCB1 except
     * that the bound is computed as \f[
     *
     * Q_{(s,a)}^+ = \widehat{Q}_{(s,a)} + C_p \sqrt{\widetilde{Q}_{(s,a)}}
     *
     * \f] where \f$\widehat{Q}_{(s,a)}\f$ is the mean value,
     * \f$\widetilde{Q}_{(s,a)}\f$ is the variance of the value, and \f$C_p\f$
     * (as in UCB1) balances exploration and exploitation. These bounds take
     * into account uncertainty further down in the tree. */
    class UCB_Plus: public MaxPolicy {
    public:
        /**
         * Constructor. @param Cp This is the scaling parameter for
         * exploration.*/
        UCB_Plus(double Cp = 1);
        virtual reward_t score(const node_t & state_node,
                               const arc_t & to_action_arc,
                               const node_t & action_node,
                               std::shared_ptr<Environment> environment,
                               const graph_t & graph,
                               const node_info_map_t & node_info_map,
                               const mcts_node_info_map_t & mcts_node_info_map,
                               const mcts_arc_info_map_t & mcts_arc_info_map) const override;
    protected:
        double Cp;
    };

} // end namespace tree_policy

#endif /* TREEPOLICY_H_ */
