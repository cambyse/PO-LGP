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
    /** Node-specific data for nodes in a SearchTree. */
    struct NodeInfo {
        /** Type of the node. */
        NODE_TYPE type = STATE_NODE;
        /** Only valid for ACTION_NODE: the action the node corresponds to. */
        action_t action = -1;
        /** Only valid for STATE_NODE: the state the node corresponds to. */
        state_t state = -1;
        /** Nr of times this node (state or action) was sampled. */
        int counts = 0;
        /** Value of this action/state. */
        double value = 0;
    };
    /** Arc-specific data for arcs in a SearchTree. */
    struct ArcInfo {
        /** Only valid for action --> state arcs: number of times this
         * transition occurred. */
        int counts = 0;
        /** Only valid for action --> state arcs: sum of all rewards for this
         * transition. */
        double mean_reward = 0;
        /** Only valid for action --> state arcs: probability for this
         * transition to occurred */
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
    /** An entry within a trajectory. A trajectory item (\e state_node_from, \e
     * arc_to_action_node, \e action_node, \e arc_to_state_node, \e
     * state_node_to) describes a complete transition from one state to the
     * next. \e state_node_to should be the same node as \e state_node_from in
     * the next trajcetory item.*/
    typedef std::tuple<node_t, arc_t, node_t, arc_t, node_t> trajectory_item_t;
    /** A trajectory. This is just a sequence if #trajectory_item_t objects
     * describing the corresponding transitions. */
    typedef std::list<trajectory_item_t>    trajectory_t;

    //----members----//
public:
    /** The seach tree. */
    graph_t graph;
    /** Map holding node-specific information of type NodeInfo . */
    node_info_map_t node_info_map;
    /** Map holding arc-specific information of type ArcInfo. */
    arc_info_map_t arc_info_map;
    /** Root node of the search tree. */
    node_t root_node = lemon::INVALID;
    /** Discount factor for computing the value. */
    double discount;
    /** Pointer to the environment. */
    std::shared_ptr<Environment> environment;
private:
    static const bool use_sqrt_scale = true;

    //----methods----//
public:
    SearchTree(const state_t &, std::shared_ptr<Environment> env, double d = 0.9);
    virtual ~SearchTree() = default;
    /** Initializes an empty search tree with the root node set to \e s. This
     * function must be called before using the class and may be used to reset
     * everything. */
    void init(const state_t & s);
    /** Performs a single rollout. */
    void perform_rollout();
    /** Returns a recommendation for an action for the root node. */
    action_t recommend_action() const;
    /** Prunes the tree according to the given action and state. This function
     * may be called after an action was actually performed in the environment
     * to reuse the relevant rest of the tree instead of resetting it with
     * init().*/
    void prune(const action_t &, const state_t &);
    /** Prints the graph to a PDF file with given name. */
    void toPdf(const char* file_name) const;
private:
    trajectory_item_t add_sample(const node_t &,
                                 const action_t &,
                                 const state_t &,
                                 const reward_t &);
    QString str(const node_t &) const;
    QString str_rich(const node_t &) const;
    double color_rescale(const double&) const;
    void update_data(const node_t   & state_node,
                     const arc_t    & state_action_arc,
                     const node_t   & action_node,
                     const arc_t    & action_state_arc,
                     const reward_t & reward);
    void update_model(const node_t  & state_node,
                      const arc_t   & state_action_arc,
                      const node_t  & action_node,
                      const arc_t   & action_state_arc);
    action_t tree_policy(const node_t &, bool &) const;
};

#endif /* SEARCHTREE_H_ */
