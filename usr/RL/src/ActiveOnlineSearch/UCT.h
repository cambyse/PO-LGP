#ifndef UCT_TREE_SEARCH_H_
#define UCT_TREE_SEARCH_H_

#include "SearchTree.h"

class UCT: public SearchTree {
    //----typedefs/classes----//
public:
    /**
     * Node-specific data for nodes in a UCT %SearchTree. This complements the
     * SearchTree::NodeInfo data stored in SeachTree::node_info_map.*/
    struct UCTNodeInfo {
        /**
         * Nr of times this node (state or action) was sampled. */
        int counts = 0;
        /**
         * Value of this action/state. */
        double value = 0;
    };
    /**
     * Arc-specific data for arcs in a UCT %SearchTree. This complements the
     * SearchTree::ArcInfo data stored in SeachTree::arc_info_map.*/
    struct UCTArcInfo {
        /**
         * Only valid for action --> state arcs: number of times this
         * transition occurred. */
        int counts = 0;
        /**
         * Only valid for action --> state arcs: sum of all rewards for this
         * transition. */
        double mean_reward = 0;
    };
    typedef graph_t::NodeMap<UCTNodeInfo> uct_node_info_map_t;
    typedef graph_t::ArcMap<UCTArcInfo>   uct_arc_info_map_t;
    /**
     * An entry within a trajectory. A trajectory item (\e state_node_from, \e
     * arc_to_action_node, \e action_node, \e arc_to_state_node, \e
     * state_node_to) describes a complete transition from one state to the
     * next. \e state_node_to should be the same node as \e state_node_from in
     * the next trajcetory item.*/
    typedef std::tuple<node_t, arc_t, node_t, arc_t, node_t> trajectory_item_t;
    /**
     * A trajectory. This is just a sequence if #trajectory_item_t objects
     * describing the corresponding transitions. */
    typedef std::list<trajectory_item_t>    trajectory_t;

    //----members----//
protected:
    /**
     * Map holding node-specific information of type UCTNodeInfo . */
    uct_node_info_map_t uct_node_info_map;
    /**
     * Map holding arc-specific information of type UCTArcInfo. */
    uct_arc_info_map_t uct_arc_info_map;

    //----methods----//
public:
    UCT(const state_t &, std::shared_ptr<Environment>, double d = 0.9);
    virtual ~UCT() = default;
    void perform_rollout() override;
    action_t recommend_action() const override;
    void toPdf(const char* file_name) const override;
protected:
    trajectory_item_t add_sample(const node_t &,
                                 const action_t &,
                                 const state_t &,
                                 const reward_t &);
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

#endif /* UCT_TREE_SEARCH_H_ */
