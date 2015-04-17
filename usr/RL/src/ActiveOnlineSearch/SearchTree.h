#ifndef SEARCHTREE_H_
#define SEARCHTREE_H_

#include <memory> // for shared_ptr

#include <lemon/list_graph.h>

#include "Environment.h"

class SearchTree {
    //----typedefs/classes----//
public:
    typedef Environment::state_t state_t;
    typedef Environment::action_t action_t;
    typedef Environment::reward_t reward_t;
    enum NODE_TYPE {STATE_NODE, ACTION_NODE};
    struct NodeInfo {
        NODE_TYPE type = STATE_NODE;
        // only valid for ACTION_NODE
        action_t action = -1;
        // only valid for STATE_NODE
        state_t state = -1;
        // nr of times this node (state or action) was sampled
        int counts = 0;
        // value of this action/state
        double value = 0;
    };
    struct ArcInfo {
        // only valid for action --> state arcs, number of times this transition
        // occurred
        int counts = 0;
        // only valid for action --> state arcs, sum of all rewards for this
        // transition
        double mean_reward = 0;
        // only valid for action --> state arcs, probability for this transition
        // to occurred
        double probability = 0;
    };
    typedef lemon::ListDigraph                graph_t;
    typedef graph_t::Node                     node_t;
    typedef graph_t::Arc                      arc_t;
    typedef graph_t::NodeIt                   node_it_t;
    typedef graph_t::ArcIt                    arc_it_t;
    typedef graph_t::InArcIt                  in_arc_it_t;
    typedef graph_t::OutArcIt                 out_arc_it_t;
    typedef graph_t::NodeMap<NodeInfo>        node_info_map_t;
    typedef graph_t::ArcMap<ArcInfo>          arc_info_map_t;
    typedef std::tuple<node_t, arc_t, node_t, arc_t, node_t> trajectory_item_t;
    typedef std::list<trajectory_item_t>    trajectory_t;

    //----members----//
public:
    graph_t graph;
    node_info_map_t node_info_map;
    arc_info_map_t arc_info_map;
    node_t root_node = lemon::INVALID;
    double discount;
    std::shared_ptr<Environment> environment;
private:
    bool use_sqrt_scale = true;

    //----methods----//
public:
    SearchTree(const state_t &, std::shared_ptr<Environment> env, double d = 0.9);
    virtual ~SearchTree() = default;
    void init(const state_t &);
    void perform_rollout();
    action_t recommend_action() const;
    void prune(const action_t &, const state_t &);
    void toPdf(const char*) const;
private:
    trajectory_item_t add_sample(const node_t &, const action_t &, const state_t &, const reward_t &);
    QString str(const node_t &) const;
    QString str_rich(const node_t &) const;
    double color_rescale(const double&) const;
    void update_data(const node_t   & state_node,
                     const arc_t    & state_action_arc,
                     const node_t   & action_node,
                     const arc_t    & action_state_arc,
                     const reward_t & reward);
    void update_model(const node_t & state_node,
                      const arc_t & state_action_arc,
                      const node_t & action_node,
                      const arc_t & action_state_arc);
    action_t tree_policy(const node_t &, bool &) const;
};

#endif /* SEARCHTREE_H_ */
