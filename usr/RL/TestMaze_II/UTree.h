#ifndef UTREE_H_
#define UTREE_H_

#include "Data.h"
#include "Representation/Representation.h"
#include "Feature.h"

#include <lemon/list_graph.h>

#include <vector>
#include <set>

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
        instance_vector_t instance_vector;
        const Feature * feature;
        f_ret_t parent_return_value;
    };

    typedef graph_t::NodeMap<NodeInfo> node_info_map_t;

    UTree();

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

    double expand_leaf_node(const double& score_threshold = 0);

private:

    enum TEST_TYPE { KOLMOGOROV_SMIRNOV, CHI_SQUARE };
    enum SCORE_TYPE { SCORE_BY_REWARDS, SCORE_BY_ACTIONS, SCORE_BY_BOTH };

    int k;
    instance_t * instance_data;
    std::vector<Feature*> basis_features;
    node_container_t leaf_nodes;
    graph_t graph;
    node_t root_node;
    node_info_map_t node_info_map;
    const int pseudo_counts = 1;

    const TEST_TYPE test_type = CHI_SQUARE;
    const SCORE_TYPE score_type = SCORE_BY_BOTH;

    node_t add_child(const node_t& node);

    void insert_instance(const instance_t *, const node_t& node, const bool& descendants_only = false);

    double score_leaf_node(const node_t leaf_node, const Feature* feature) const;

    double sample_size_factor(const int& n) const;

    node_t find_leaf_node(const instance_t *) const;

    probability_t prior_probability(const state_t&, const reward_t&) const;

};

#endif /* UTREE_H_ */
