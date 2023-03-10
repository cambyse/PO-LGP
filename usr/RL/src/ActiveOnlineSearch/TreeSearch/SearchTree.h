#ifndef SEARCHTREE_H_
#define SEARCHTREE_H_

#include "AbstractSearchTree.h"

#include <tuple>
#include <unordered_map>
#include <list>

#include <lemon/list_graph.h>

#include <QString>

namespace node_finder {
    class NodeFinder;
};

class SearchTree: public AbstractSearchTree {
    //----typedefs/classes----//
public:
    enum NODE_TYPE {OBSERVATION_NODE, ACTION_NODE};
    typedef lemon::ListDigraph graph_t;
    typedef graph_t::Node      node_t;
    typedef graph_t::Arc       arc_t;
    typedef graph_t::NodeIt    node_it_t;
    typedef graph_t::ArcIt     arc_it_t;
    typedef graph_t::InArcIt   in_arc_it_t;
    typedef graph_t::OutArcIt  out_arc_it_t;
    typedef std::tuple<arc_t,node_t,bool,bool> arc_node_t;
    /**
     * Basic information about a node. That is, its type (observation or action) and
     * the corresponding observation or action, respectively. */
    struct NodeInfo {
        /**
         * Type of the node. */
        NODE_TYPE type = OBSERVATION_NODE;
        /**
         * Only valid for ACTION_NODE: the action the node corresponds to. */
        action_handle_t action = nullptr;
        /**
         * Only valid for OBSERVATION_NODE: the observation the node corresponds to. */
        observation_handle_t observation = nullptr;
    };
    typedef graph_t::NodeMap<NodeInfo> node_info_map_t;

    //----members----//
protected:
    /**
     * Whether to use square-root scale for colors in PDF output. */
    static const bool use_sqrt_scale = false;
    /**
     * Class for identifying existing nodes. */
    const std::shared_ptr<node_finder::NodeFinder> node_finder;
    /**
     * The seach tree. */
    graph_t graph;
    /**
     * Root node of the search tree. */
    node_t root_node = lemon::INVALID;
    /**
     * Map holding node-specific information of type NodeInfo. */
    node_info_map_t node_info_map;

    //----methods----//
public:
    SearchTree(std::shared_ptr<AbstractEnvironment> environment,
               double discount,
               std::shared_ptr<node_finder::NodeFinder> node_finder);
    virtual ~SearchTree() = default;
    virtual void init() override;
    /**
     * Calls init() if necessary, then calls next_do() */
    virtual void next() override final;
    virtual void next_do() = 0;
    virtual void update(const action_handle_t &,
                       const observation_handle_t &) override;
    virtual void plot_graph(const char* file_name,
                            const char* command,
                            const char* parameters,
                            bool delete_dot_file) const override;
    virtual const graph_t & get_graph() const;
    virtual const node_info_map_t & get_node_info_map() const;
protected:
    virtual QString str_html(const node_t &) const;
    /**
     * Tries to find an existing observation node or creates one. Returns the
     * arc to the node, the node itself, and whether arc or node were newly
     * created. */
    virtual arc_node_t find_or_create_observation_node(const node_t & action_node,
                                                       const observation_handle_t & observation);
    /**
     * Tries to find an existing action node or creates one. Returns the arc to
     * the node, the node itself, and whether arc or node were newly created. */
    virtual arc_node_t find_or_create_action_node(const node_t & observation_node,
                                                  const action_handle_t & action);
    virtual arc_node_t add_observation_node(observation_handle_t observation, node_t action_node);
    virtual arc_node_t add_action_node(action_handle_t action, node_t observation_node);
    virtual void erase_node(node_t);
};

#endif /* SEARCHTREE_H_ */
