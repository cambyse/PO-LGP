#ifndef UTREE_H_
#define UTREE_H_

#include "Data.h"
#include "Representation/Representation.h"
#include "Feature.h"

#include <lemon/list_graph.h>

#include <vector>

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

private:

    int k;
    instance_t * instance_data;
    std::vector<Feature*> basis_features;
    graph_t graph;
    node_t root_node;
    node_info_map_t node_info_map;

    void insert_instance(const instance_t *, const node_t& node);
};

#endif /* UTREE_H_ */
