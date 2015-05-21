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
    typedef node_finder::NodeFinder NodeFinder;
    typedef std::tuple<arc_t,node_t> arc_node_t;
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
    const std::shared_ptr<NodeFinder> node_finder;
    /**
     * The seach tree. */
    graph_t graph;
    /**
     * Root node of the search tree. */
    node_t root_node = lemon::INVALID;
    /**
     * Root state of the search tree. */
    state_handle_t root_state = nullptr;
    /**
     * Map holding node-specific information of type NodeInfo. */
    node_info_map_t node_info_map;

    //----methods----//
public:
    SearchTree(std::shared_ptr<AbstractEnvironment> environment,
               double discount,
               std::shared_ptr<NodeFinder> node_finder);
    virtual ~SearchTree() = default;
    virtual void init(const state_handle_t & root_state) override;
    /**
     * Calls init() if necessary, then calls next_do() */
    virtual void next() override final;
    virtual void next_do() = 0;
    virtual void prune(const action_handle_t &,
                       const observation_handle_t &,
                       const state_handle_t &) override;
    virtual void toPdf(const char* file_name) const override;
    /* /\** */
    /*  * Get const reference to root node. *\/ */
    /* virtual const node_t & get_root_node() const final; */
    /* /\** */
    /*  * Get handle for root state. *\/ */
    /* virtual const state_handle_t & get_root_state() const final; */
    /* /\** */
    /*  * Get const reference to graph. *\/ */
    /* virtual const graph_t & get_graph() const final; */
    /* /\** */
    /*  * Get const reference to node_info_map. *\/ */
    /* virtual const node_info_map_t & get_node_info_map() const final; */
    /* /\** */
    /*  * Get observation of a node. *\/ */
    /* virtual const observation_handle_t observation(const node_t & observation_node) const final; */
    /* /\** */
    /*  * Get action of a node. *\/ */
    /* virtual const action_handle_t action(const node_t & action_node) const final; */
    /* /\** */
    /*  * Get type of a node. *\/ */
    /* virtual const NODE_TYPE type(const node_t & node) const final; */
protected:
    /* /\** */
    /*  * Counts the number of children by iterating over outgoing arcs. *\/ */
    /* virtual size_t number_of_children(const node_t & observation_node) const; */
    /* /\** */
    /*  * Returns \c true \e iff number_of_children() is equal to */
    /*  * environment.actions.size(). *\/ */
    /* virtual bool is_fully_expanded(const node_t & observation_node) const; */
    /* /\** */
    /*  * Returns \c true \e iff number_of_children() is larger than zero but */
    /*  * smaller than environment.actions.size(). *\/ */
    /* virtual bool is_partially_expanded(const node_t & observation_node) const; */
    /* /\** */
    /*  * Returns \c true \e iff number_of_children() is zero. This function does */
    /*  * not call number_of_children() to avoid iterating over all arcs. *\/ */
    /* virtual bool is_not_expanded(const node_t & observation_node) const; */
    /* virtual QString str(const node_t &) const; */
    virtual QString str_html(const node_t &) const;
    virtual arc_node_t find_or_create_observation_node(const node_t & action_node,
                                                                     const observation_handle_t & observation);
    virtual arc_node_t find_or_create_action_node(const node_t & observation_node,
                                                                const action_handle_t & action);
    virtual arc_node_t add_observation_node(observation_handle_t observation, node_t action_node);
    virtual arc_node_t add_action_node(action_handle_t action, node_t observation_node);
    virtual void erase_node(node_t);
/* private: */
/*     arc_node_t find_observation_node_among_children(const node_t & action_node, */
/*                                                                   const observation_handle_t & observation) const; */
/*     node_t find_observation_node_among_siblings_children(const node_t & action_node, */
/*                                                          const observation_handle_t & observation) const; */
/*     node_t find_observation_node_at_same_depth(const node_t & action_node, */
/*                                                const observation_handle_t & observation) const; */
/*     void add_observation_node_to_level_map(const node_t & observation_node); */
};

#endif /* SEARCHTREE_H_ */
