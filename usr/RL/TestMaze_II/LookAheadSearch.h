#ifndef LOOKAHEADSEARCH_H_
#define LOOKAHEADSEARCH_H_

#include <lemon/list_graph.h>
#include <lemon/graph_to_eps.h>
#include <lemon/dim2.h>
#include <lemon/connectivity.h>
#include <lemon/adaptors.h>

#include "Config.h"
#include "util/ProgressBar.h"

#include "Predictor.h"

class Environment;

class LookAheadSearch {

public:

    USE_CONFIG_TYPEDEFS;
    typedef AbstractReward::value_t value_t;

    enum NODE_TYPE { NONE, OBSERVATION, ACTION };
    enum EXPANSION_TYPE { NOT_DEFINED, NOT_EXPANDED, FULLY_EXPANDED };

    struct NodeInfo {
        NodeInfo();
        NodeInfo(
                const NODE_TYPE&,
                const EXPANSION_TYPE&,
                instance_t *,
                const action_ptr_t&,
                const value_t&,
                const value_t&
        );
        NodeInfo(const NodeInfo&);
        ~NodeInfo();
        NodeInfo& operator=(const NodeInfo&);
        NODE_TYPE type;
        EXPANSION_TYPE expansion;
        instance_t * instance; // !!!Needs to be set and deleted manually!!!
        action_ptr_t action;
        value_t upper_value_bound, lower_value_bound;
    };

    struct ArcInfo {
        ArcInfo(const reward_ptr_t& r = 0, const probability_t& p = 0): transition_reward(r), prob(p) {}
        reward_ptr_t transition_reward;
        probability_t prob;
    };

    typedef lemon::ListDigraph                graph_t;
    typedef graph_t::Node                     node_t;
    typedef graph_t::Arc                      arc_t;
    typedef graph_t::NodeMap<NodeInfo>        node_info_map_t;
    typedef graph_t::ArcMap<ArcInfo>          arc_info_map_t;
    typedef graph_t::NodeMap<lemon::Color>    node_color_map_t;

    typedef std::vector<node_t> node_vector_t;

    LookAheadSearch(const double& d);
    virtual ~LookAheadSearch();

    /** \brief Initialize action, observation, and reward spaces. */
    virtual void set_spaces(const Environment & environment);

    /** \brief Set the spaces used for planning. */
    void set_spaces(const action_ptr_t & a, const observation_ptr_t & o, const reward_ptr_t & r);

    /*! \brief Clears the search tree. */
    void clear_tree();

    /*! \brief Build a search tree from the root observation. */
    void build_tree(
            const instance_t * root,
            const Predictor& environment,
            const size_t& max_node_counter = 0
    );

    /*! \brief Expand current tree by expanding one leaf
     * node and back-propagating changes. Return whether
     * tree needs further expansion. */
    bool expand_tree(const Predictor&);

    /*! \brief Repeatedly expand tree until either the optimal action is
     *  unambiguous or the maximum tree size is reached. */
    void fully_expand_tree(
            const Predictor& environment,
            const size_t& max_node_counter = 0
    );

    /*! \brief Returns the best action for the root observation. */
    action_ptr_t get_optimal_action() const;

    /*! \brief Returns the predicted transition probability for given action to
     *  given observation and reward. */
    probability_t get_predicted_transition_probability(const action_ptr_t&,
                                                       const observation_ptr_t&,
                                                       const reward_ptr_t& ,
                                                       const Predictor& environment
        ) const;

    /*! \brief Prune obsolete branches after performing action a into observation s
     *  and reset root node. */
    void prune_tree(const action_ptr_t& a, const instance_t * new_root_instance, const Predictor& environment);

    /*! \brief Set the discount rate used for computing observation and action values. */
    void set_discount(const double& d) { discount = d; }

    /*! \brief Print the tree to console and/or as eps file. */
    void print_tree(const bool& text,
                    const bool& eps_export,
                    const char* file_name = "look_ahead_tree.eps",
                    const node_color_map_t * color_map = nullptr
        ) const;

    /*! \brief Print the tree statistics to console. */
    void print_tree_statistics();

    size_t get_number_of_nodes() const { return number_of_nodes; }

protected:

    typedef lemon::dim2::Point<double> Point;

    graph_t graph;
    node_t root_node;
    node_info_map_t node_info_map;
    arc_info_map_t arc_info_map;
    double discount;
    size_t number_of_nodes;
    static const bool random_tie_break;

    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;

    /*! \brief Sets the way the upper and lower bounds
     * are used for node selection and back-propagation. */
    enum BOUND_USAGE_TYPE {

        /*! Use maximum upper bound. */
        MAX_UPPER_BOUND,

        /*! Use maximum lower bound. */
        MAX_LOWER_BOUND,

        /*! Use maximum weighted uncertainty. */
        MAX_WEIGHTED_UNCERTAINTY,

        /*! Use expected lower and upper bounds. */
        EXPECTED_BOUNDS,

        /*! Use maximum upper bound for the upper bound and maximum lower bound
         *  for lower bound. This is the 'optimistic' or best-case assumption
         *  for estimating the bounds. */
        MAX_UPPER_FOR_UPPER_MAX_LOWER_FOR_LOWER,

        /*! Use maximum upper bound for the upper bound and minimum lower bound
         *  for lower bound. This is the worst-case assumption in terms of
         *  certainty, the bounds are as wide as possible. */
        MAX_UPPER_FOR_UPPER_MIN_LOWER_FOR_LOWER,

        /*! Use maximum upper bound for the upper bound and the corresponding lower bound for lower bound. */
        MAX_UPPER_FOR_UPPER_CORRESPONDING_LOWER_FOR_LOWER,

        /*! Use weighted bounds for calculating value:
         * \f$ value = \alpha \cdot bound_{lower} + (1-\alpha) \cdot bound_{upper} \f$
         * with \f$\alpha = \f$ LookAheadSearch::lower_bound_weight.\n\n
         * For binary rewards \f$ r_{min} \f$ and \f$ r_{max} \f$ with
         * probabilities \f$ p_{min} \f$ and \f$ p_{max} \f$,
         * \f$ p_{min} \cdot r_{min} + p_{max} \cdot r_{max} \f$
         * corresponds to the the expected reward in every time step.
         * Given upper and lower bounds
         * \f$value = p_{min} \cdot bound_{lower} + (1-p_{min}) \cdot bound_{upper}\f$
         * equals the reinforcement value i.e. the expected reward discounted over time.*/
        MAX_WEIGHTED_BOUNDS,

        /*! Use the same strategy as is used for optimal action selection (defined by LookAheadSearch::optimal_action_selection_type). */
        SAME_AS_OPTIMAL_ACTION_SELECTION
    };

    /*! \brief Weight for lower bound in strategy
     * LookAheadSearch::MAX_WEIGHTED_BOUNDS.
     *
     * For binary rewards this corresponds to the
     * prior probability of obtaining the lower reward. */
    static const double lower_bound_weight;

    /*! \brief Defines how the optimal action is determined.
     *
     * Currently LookAheadSearch::MAX_LOWER_BOUND and MAX_WEIGHTED_BOUNDS are supported. */
    static const BOUND_USAGE_TYPE optimal_action_selection_type = MAX_WEIGHTED_BOUNDS;

    /*! \brief Defines how action nodes are selected for further examination.
     *
     * Currently LookAheadSearch::MAX_UPPER_BOUND and LookAheadSearch::MAX_LOWER_BOUND are supported. */
    static const BOUND_USAGE_TYPE tree_action_selection_type    = MAX_UPPER_BOUND;

    /*! \brief Defines how observation nodes are selected for further examination.
     *
     * Currently only LookAheadSearch::MAX_WEIGHTED_UNCERTAINTY is
     * supported. LookAheadSearch::MAX_WEIGHTED_UNCERTAINTY means that the
     * uncertainty of each observation is multiplied with the probability of reaching
     * this observation by executing the action. This product -- the weighted
     * uncertainty -- is evaluated to determine the observation with a maximum
     * weighted uncertainty, which is further explored. */
    static const BOUND_USAGE_TYPE tree_observation_selection_type     = MAX_WEIGHTED_UNCERTAINTY;

    /*! \brief Defines how observation bounds are used for calculating
     * action bounds in back-propagation.
     *
     * Currently only LookAheadSearch::EXPECTED_BOUNDS and
     * LookAheadSearch::MAX_UPPER_FOR_UPPER_MIN_LOWER_FOR_LOWER are
     * supported. */
    static const BOUND_USAGE_TYPE action_back_propagation_type  = EXPECTED_BOUNDS;

    /*! \brief Defines how action bounds are used for calculating
     * observation bounds in back-propagation.
     *
     * Currently
     * LookAheadSearch::MAX_UPPER_FOR_UPPER_CORRESPONDING_LOWER_FOR_LOWER,
     * LookAheadSearch::MAX_WEIGHTED_BOUNDS, and
     * LookAheadSearch::SAME_AS_OPTIMAL_ACTION_SELECTION are supported
     * (LookAheadSearch::optimal_action_selection_type must match on of the
     * implemented strategies). In the case of
     * LookAheadSearch::MAX_WEIGHTED_BOUNDS the action node with highes weighted
     * bounds is chosen and the bounds of this action node are used for the
     * observation node. */
    static const BOUND_USAGE_TYPE observation_back_propagation_type   = SAME_AS_OPTIMAL_ACTION_SELECTION;

    /*! \brief Select the next action node for finding the leaf that is to be expanded. */
    node_t select_next_action_node(node_t observation_node);

    /*! \brief Select the next observation node (possibly a leaf) for finding the leaf that is to be expanded. */
    node_t select_next_observation_node(node_t action_node);

    /*! \brief Expand the leaf node given a predictive environment. */
    void expand_leaf_node(
            node_t observation_node,
            const Predictor& environment
    );

    /*! \brief Expand the action node given a predictive environment. */
    void expand_action_node(
            node_t action_node,
            const Predictor& environment
    );

    /** \brief Select the optimal action from a given observation node. */
    node_vector_t optimal_action_nodes(const node_t& observation_node) const;

    /*! \brief Update the given action node when back-propagating after leaf expansion.
     * Return parent observation node. */
    node_t update_action_node(node_t action_node);

    /*! \brief Update the given observation node when back-propagating after leaf expansion.
     * Return parent action node. */
    node_t update_observation_node(node_t observation_node);

    /*! \brief Returns whether the root_node needs further expansion.
     *
     * If there is an action with lower value bound greater than the
     * upper value bounds of all other actions this action cannot
     * become sub-optimal by further expansion of the tree.
     */
    bool tree_needs_further_expansion();

    /*! \brief Upper bound for observation value without prior knowledge. */
    value_t get_upper_value_bound() { return reward_space->max_reward()/(1-discount); }

    /*! \brief Lower bound for observation value without prior knowledge. */
    value_t get_lower_value_bound() { return reward_space->min_reward()/(1-discount); }

    /*! \brief Print information on node. */
    void print_node(node_t node) const;

    /*! \brief Node energy function. */
    double node_energy(node_t node, const graph_t::NodeMap<Point>& coords) const;

};

#endif /* LOOKAHEADSEARCH_H_ */
