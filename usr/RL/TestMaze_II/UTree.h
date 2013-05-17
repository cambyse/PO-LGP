#ifndef UTREE_H_
#define UTREE_H_

#include "Data.h"
#include "Representation/Representation.h"
#include "Feature.h"

#include <lemon/list_graph.h>

#include <vector>
#include <set>
#include <map>

class UTree
{
public:

    typedef Data::idx_t                      idx_t;
    typedef Data::size_t                     size_t;
    typedef Data::probability_t              probability_t;
    typedef Feature::feature_return_value    f_ret_t;
    typedef lemon::ListDigraph               graph_t;
    typedef graph_t::Node                    node_t;
    typedef graph_t::Arc                     arc_t;
    typedef std::vector<node_t>              node_vector_t;
    typedef std::set<node_t>                 node_container_t;
    typedef std::vector<const instance_t *>  instance_vector_t;

    struct NodeInfo {
        NodeInfo(const Feature * f = nullptr, const f_ret_t& r = f_ret_t());
        instance_vector_t instance_vector;                                      // data
        const Feature * feature;                                                // discriminating feature for non-leaf nodes
        f_ret_t parent_return_value;                                            // return value of parent-node's feature

        std::map<const Feature*,double> scores;                                 // the scores for different features
        bool scores_up_to_date;                                                 // whether leaf-node's scores are up-to-date

        std::map<action_t,double> state_action_values;                          // Q(s,a)-function
        double max_state_action_value;                                          // utility / state-value
        action_t max_value_action;                                              // policy
        std::map< std::pair<action_t,node_t>, probability_t > transition_table; // state transition table
        std::map< std::pair<action_t,node_t>, double > expected_reward;         // expected reward
        bool statistics_up_to_date;                                             // whether the above is up-to-date
    };

    typedef graph_t::NodeMap<NodeInfo> node_info_map_t;

    enum EXPANSION_TYPE { UTILITY_EXPANSION, STATE_REWARD_EXPANSION };

    UTree(const double&);

    virtual ~UTree();

    void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );

    void clear_data();

    probability_t get_prediction(const instance_t *, const action_t&, const state_t&, const reward_t&) const;
    probability_t (UTree::*get_prediction_ptr())(const instance_t *, const action_t&, const state_t&, const reward_t&) const {
        return &UTree::get_prediction;
    }

    void print_tree();
    void print_leaves();
    void clear_tree();

    double expand_leaf_node(const double& score_threshold = 0);

    double q_iteration(const double& alpha);

    double value_iteration();

    action_t get_max_value_action(const instance_t *);

    /*! \brief Set the discount rate used for computing state and action values. */
    void set_discount(const double& d) { discount = d; }

    void set_expansion_type(const EXPANSION_TYPE& ex) { expansion_type = ex; }
    EXPANSION_TYPE get_expansion_type() const { return expansion_type; }

private:

    int k;
    instance_t * instance_data;
    std::vector<Feature*> basis_features;
    node_container_t leaf_nodes;
    graph_t graph;
    node_t root_node;
    node_info_map_t node_info_map;
    const int pseudo_counts = 1;
    double discount;

    EXPANSION_TYPE expansion_type = UTILITY_EXPANSION;

    node_t add_child(const node_t& node);

    void insert_instance(const instance_t *, const node_t& node, const bool& descendants_only = false);

    double score_leaf_node(const node_t leaf_node, const Feature* feature) const;

    double sample_size_factor(const int& n1, const int& n2) const;

    node_t find_leaf_node(const instance_t *) const;

    probability_t prior_probability(const state_t&, const reward_t&) const;

    void update_statistics(const node_t& leaf_node);

    void update_state_value_and_policy(const node_t& leaf_node);
};

#endif /* UTREE_H_ */
