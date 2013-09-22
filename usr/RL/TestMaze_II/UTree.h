#ifndef UTREE_H_
#define UTREE_H_

#include "Config.h"

#include "Feature.h"
#include "HistoryObserver.h"

#include <lemon/list_graph.h>

#include <vector>
#include <set>
#include <map>

class UTree: public HistoryObserver
{
private:
    struct NodeInfo; // forward declaration of private type

public:
    USE_CONFIG_TYPEDEFS;
    typedef Feature::feature_return_value    f_ret_t;
    typedef lemon::ListDigraph               graph_t;
    typedef graph_t::Node                    node_t;
    typedef graph_t::Arc                     arc_t;
    typedef std::vector<node_t>              node_vector_t;
    typedef std::set<node_t>                 node_container_t;
    typedef std::vector<const instance_t *>  instance_vector_t;

    /** \brief Type of the map storing the NodeInfo objects. */
    typedef graph_t::NodeMap<NodeInfo> node_info_map_t;

    /** \brief Expand the tree to represent a value function or to make
     * predictions. */
    enum EXPANSION_TYPE { UTILITY_EXPANSION, STATE_REWARD_EXPANSION };

    UTree(const double&);
    virtual ~UTree();

    /** \brief Add a new instance to the tree. */
    virtual void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward,
            const bool& new_episode
    );

    /** \brief Clears all data (all instances) from tree, but does not clear the
     * tree. */
    virtual void clear_data();

    /** \brief Returns a prediction of how probable the state and reward are
     * give the instance and action. */
    probability_t get_prediction(const instance_t *, const action_t&, const state_t&, const reward_t&) const;
    /** \brief Return function pointer to be used by LookAheadSearch. */
    probability_t (UTree::*get_prediction_ptr())(const instance_t *, const action_t&, const state_t&, const reward_t&) const {
        return &UTree::get_prediction;
    }

    /** \brief Print the tree nodes and associated information. */
    void print_tree();
    /** \brief Print only the leaf nodes and associated information. */
    void print_leaves();
    /** \brief Clear the tree but not the data (instances). */
    void clear_tree();

    /** \brief Return the current tree size. */
    int get_tree_size() const;

    /** \brief Expand the highest scored leaf by using corresponding feature.
     *
     * Scores are updated if necessary. For every leaf-feature pair a score is
     * computed. If the highes score is not less than score_threshold, the
     * correpsonding leaf is expanded. */
    double expand_leaf_node(const double& score_threshold = 0);

    double q_iteration(const double& alpha);

    double value_iteration();

    action_t get_max_value_action(const instance_t *);

    /*! \brief Set the discount rate used for computing state and action values. */
    void set_discount(const double& d) { discount = d; }

    /** \brief Set the expansion type. */
    void set_expansion_type(const EXPANSION_TYPE& ex);
    /** \brief Get the expansion type. */
    EXPANSION_TYPE get_expansion_type() const { return expansion_type; }

private:

    /** \brief Stores data associated with the single nodes.
     *
     * Scores, values, and statistics need to be updated in mutual dependence
     * and depending on new data and leaf-expansion. Assuming a fully connected
     * transition structur, all values are couples and are thus up-to-date or
     * invalid all at the same time. @code
     *
     *  =====> leaf-expansion <-----------------\
     *              |   \                        \
     *              |    \ (new children only)    \
     *              |     V                        \
     *  (all nodes) |   new data <=======           \
     *              |    /                           |
     *              |   / (affected leaves only)     |
     *              |  /                             |
     *              V V                              |
     *           statistics                          |
     *           /    |                              |
     *     (VB) /     |                              |
     *         /      |                              |
     *  /-->  V       | (affected leaves only)       |
     * |    values    | (SR)                         |
     *  \--<   \      |                              |
     *          \     |                              |
     *      (VB) \    |                             /
     *            V   V                            /
     *            scores -------------------------/
     *
     * @endcode In case of new data the statistics of the affected leaf nodes
     * need to be updated. New statistics imply a change of state-action values
     * (for value base (VB) UTree) and scores (for state-reward (SR) prediction
     * UTree) for the affected leaf nodes. Changed values affect the scores and
     * values of all other leaves (VB UTree only). Leaf expansion inserts new
     * data to the new child nodes, which changes its statistics, and also
     * modifies the transition structure (and therefore the statistics of ALL
     * nodes). */
    struct NodeInfo {
        NodeInfo(const Feature * f = nullptr, const f_ret_t& r = f_ret_t());
        instance_vector_t instance_vector;                                      ///< data
        const Feature * feature;                                                ///< discriminating feature for non-leaf nodes
        f_ret_t parent_return_value;                                            ///< return value of parent-node's feature

        std::map<const Feature*,double> scores;                                 ///< the scores for different features
        bool scores_up_to_date;                                                 ///< whether leaf-node's scores are up-to-date

        std::map<action_t,double> state_action_values;                          ///< Q(s,a)-function
        double max_state_action_value;                                          ///< utility / state-value
        action_t max_value_action;                                              ///< policy

        std::map< std::pair<action_t,node_t>, probability_t > transition_table; ///< state transition table
        std::map< std::pair<action_t,node_t>, double > expected_reward;         ///< expected reward
        bool statistics_up_to_date;                                             ///< whether the preceding two are up-to-date
    };

    std::vector<Feature*> basis_features;
    node_container_t leaf_nodes;
    graph_t graph;
    node_t root_node;
    node_info_map_t node_info_map;
    static const int pseudo_counts = 1;
    double discount;
    EXPANSION_TYPE expansion_type;

    /** \brief Whether for all nodes NodeInfo::state_action_values,
     * NodeInfo::max_state_action_value, and NodeInfo::max_value_action are
     * up-to-date. */
    bool values_up_to_date;

    /** \brief Sets NodeInfo::scores_up_to_date to false and takes care of the
     * consequences (other things not up-to-data). */
    void invalidate_scores(const node_t leaf);

    /** \brief Assert the scores are up-to-date. */
    void assert_scores_up_to_date(const node_t leaf);

    /** \brief Sets values_up_to_date to false and takes care of the
     * consequences (other things not up-to-data). */
    void invalidate_values();

    /** \brief Assert the values are up-to-date. */
    void assert_values_up_to_date();

    /** \brief Sets NodeInfo::statistics_up_to_date to false and takes care of
     * the consequences (other things not up-to-data). */
    void invalidate_statistics(const node_t leaf);

    /** \brief Assert the statistics are up-to-date. */
    void assert_statistics_up_to_date(const node_t leaf);

    node_t add_child(const node_t& node);

    void insert_instance(const instance_t *, const node_t& node, const bool& descendants_only = false);

    double score_leaf_node(const node_t leaf_node, const Feature* feature) const;

    /** \brief Weight factor for considering the sample size.
     *
     * Small samples sizes reduce the reliability of the statistical test. The
     * scores are therefore additionally weighted by the factor returned by this
     * function. */
    double sample_size_factor(const int& n1, const int& n2) const;

    node_t find_leaf_node(const instance_t *) const;

    probability_t prior_probability(const state_t&, const reward_t&) const;

    /** \brief Updates expected_reward and transition_table based on the
     * instance data. */
    void update_statistics(const node_t& leaf_node);
};

#endif /* UTREE_H_ */
