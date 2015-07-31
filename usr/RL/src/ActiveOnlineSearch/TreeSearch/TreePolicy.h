#ifndef TREEPOLICY_H_
#define TREEPOLICY_H_

#include "MonteCarloTreeSearch.h"

namespace tree_policy {

    typedef MonteCarloTreeSearch::graph_t              graph_t;
    typedef MonteCarloTreeSearch::node_info_map_t      node_info_map_t;
    typedef MonteCarloTreeSearch::mcts_node_info_map_t mcts_node_info_map_t;
    typedef MonteCarloTreeSearch::mcts_arc_info_map_t  mcts_arc_info_map_t;
    typedef MonteCarloTreeSearch::node_t               node_t;
    typedef MonteCarloTreeSearch::arc_t                arc_t;
    typedef AbstractEnvironment::action_handle_t       action_handle_t;
    typedef AbstractEnvironment::observation_handle_t  observation_handle_t;
    typedef AbstractEnvironment::reward_t              reward_t;
    typedef AbstractEnvironment::action_container_t    action_container_t;

    /**
     * Abstract basis class for tree policies. The job of the tree policy is to
     * traverse the existing search tree until reaching a leaf node. The most
     * common tree policy is UCB1 but a Uniform policy may also be used. */
    class TreePolicy {
    public:
        //----typedefs/classes----//
        typedef std::tuple<action_container_t,std::vector<double>> action_probability_t;
        //----members----//
        std::shared_ptr<AbstractEnvironment> environment = nullptr;
        const graph_t * graph = nullptr;
        const node_info_map_t * node_info_map = nullptr;
        const mcts_node_info_map_t * mcts_node_info_map = nullptr;
        const mcts_arc_info_map_t * mcts_arc_info_map = nullptr;
        bool restrict_to_existing = false;
    public:
        //----methods----//
        virtual ~TreePolicy() = default;
        virtual void init(std::shared_ptr<AbstractEnvironment> environment,
                          const graph_t & graph,
                          const node_info_map_t & node_info_map,
                          const mcts_node_info_map_t & mcts_node_info_map,
                          const mcts_arc_info_map_t & mcts_arc_info_map);
        virtual action_probability_t get_action_probabilities(const node_t & state_node) const = 0;
        virtual action_handle_t get_action(const node_t & state_node) const final;
        friend std::ostream& operator<<(std::ostream & out, const TreePolicy & policy) {
            policy.write(out);
            return out;
        }
        virtual void write(std::ostream &) const = 0;
    };

    /**
     * Sample actions uniformly from available action nodes. */
    class Uniform: public TreePolicy {
    public:
        virtual ~Uniform() = default;
        virtual action_probability_t get_action_probabilities(const node_t & state_node) const override;
        virtual void write(std::ostream & out) const override {out<<"Uniform()";};
    };

    /**
     * Basis class for policies that choose the action by maximizing some
     * quantity (like value or upper bound). */
    class MaxPolicy: public TreePolicy {
        graph_t::NodeMap<action_container_t> * available_actions = nullptr;
    public:
        virtual ~MaxPolicy();
        virtual void init(std::shared_ptr<AbstractEnvironment> environment,
                          const graph_t & graph,
                          const node_info_map_t & node_info_map,
                          const mcts_node_info_map_t & mcts_node_info_map,
                          const mcts_arc_info_map_t & mcts_arc_info_map) override;
        virtual action_probability_t get_action_probabilities(const node_t & state_node) const override;
        virtual double score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const = 0;
        bool print_choice = false;
        double soft_max_temperature = 0;
    };

    /**
     * Sample the action with highes value. */
    class Optimal: public MaxPolicy {
    public:
        virtual double score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const override;
        virtual void write(std::ostream & out) const override {out<<"Optimal(T="<<soft_max_temperature<<")";}
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
        virtual double score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const override;
        virtual void set_exploration(double ex) {Cp = ex;}
        virtual void write(std::ostream & out) const override {out<<"UCB1(Cp="<<Cp<<";T="<<soft_max_temperature<<")";}
    protected:
        double Cp;
    };

    /**
     * Tree policy that uses the variance of return to compute the upper bound.
     * This is similar to UCB1 except that the bound is computed as
     *
     * \f[
     * Q_{(s,a)}^+ = \widehat{Q}_{(s,a)} + \sqrt{\frac{2V\zeta\log n}{n_j}} + c \frac{3b\zeta\log n}{n_j}
     * \f]
     *
     * where V is the variance of the return (not of the value!), b is the upper
     * bound on the reward (assuming zero as lower bound), and zeta and c > 0
     * control the behavior. */
    class UCB_Variance: public MaxPolicy {
    public:
        /**
         * Constructor. @param c This is the scaling parameter for
         * exploration.*/
        UCB_Variance(double zeta = 1.2, double c = 1);
        virtual double score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const override;
        virtual void set_exploration(double ex) {c = ex;}
        virtual void write(std::ostream & out) const override {out<<"UCB_Variance(zeta=" << zeta << ";c="<<c<<";b="<<reward_bound()<<";T="<<soft_max_temperature<<")";}
        virtual double reward_bound() const;
    protected:
        double zeta, c;
    };

    class HardUpper: public MaxPolicy {
    public:
        virtual double score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const override;
        virtual void write(std::ostream & out) const override {out<<"HardUpper(T="<<soft_max_temperature<<")";}
    };

    /**
     * Uses the full statistics to compute quantiles. */
    class Quantile: public MaxPolicy {
    public:
        Quantile(double Cp,
                 double quantile,
                 double min_return = 0,
                 double max_return = 0,
                 double prior_counts = 0);
        virtual double score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const override;
        virtual void set_exploration(double ex) {Cp = ex;}
        virtual void write(std::ostream & out) const override {out<<"Quantile(Cp="<<Cp<<";q="<<quantile<<";T="<<soft_max_temperature<<")";}
        double Cp, quantile, min_return, max_return, prior_counts;
    };

} // end namespace tree_policy

#endif /* TREEPOLICY_H_ */
