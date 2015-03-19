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
    typedef lemon::ListDigraph                graph_t;
    typedef graph_t::Node                     node_t;
    typedef graph_t::Arc                      arc_t;
    typedef graph_t::NodeIt                   node_it_t;
    typedef graph_t::ArcIt                    arc_it_t;
    typedef graph_t::InArcIt                  in_arc_it_t;
    typedef graph_t::OutArcIt                 out_arc_it_t;

    /**
     * Basic information about a node. That is, its type (state or action) and
     * the corresponding state or action, respectively. */
    struct NodeInfo {
        /**
         * Type of the node. */
        NODE_TYPE type = STATE_NODE;
        /**
         * Only valid for ACTION_NODE: the action the node corresponds to. */
        action_t action = -1;
        /**
         * Only valid for STATE_NODE: the state the node corresponds to. */
        state_t state = -1;
    };
    typedef graph_t::NodeMap<NodeInfo> node_info_map_t;

    //----members----//
protected:
    /**
     * The seach tree. */
    graph_t graph;
    /**
     * Root node of the search tree. */
    node_t root_node = lemon::INVALID;
    /**
     * Map holding node-specific information of type NodeInfo . */
    node_info_map_t node_info_map;
    /**
     * Discount factor for computing the value. */
    double discount;
    /**
     * Pointer to the environment. */
    std::shared_ptr<Environment> environment;

    /**
     * Whether to use square-root scale for colors in PDF output. */
    static const bool use_sqrt_scale = true;

    //----methods----//
public:
    SearchTree(const state_t &, std::shared_ptr<Environment> env, double d = 0.9);
    virtual ~SearchTree() = default;
    /**
     * Initializes an empty search tree with the root node set to \e s. This
     * function is called by the constructor and may also be used to reset
     * everything. */
    virtual void init(const state_t & s);
    /**
     * Proceed with planning. In MonteCarloTreeSearch methods this will initiate
     * a new rollout. */
    virtual void next() = 0;
    /**
     * Returns a recommendation for an action for the root node. */
    virtual action_t recommend_action() const = 0;
    /**
     * Prunes the tree according to the given action and state. This function
     * may be called after an action was actually performed in the environment
     * to reuse the relevant rest of the tree instead of resetting it with
     * init().*/
    virtual void prune(const action_t &, const state_t &);
    /**
     * Prints the graph to a PDF file with given name. Internally uses
     * util::graph_to_pdf(). */
    virtual void toPdf(const char* file_name) const;
protected:
    virtual bool is_leaf(const node_t & n) const {return out_arc_it_t(graph,n)==lemon::INVALID;}
    virtual QString str(const node_t &) const;
    virtual QString str_rich(const node_t &) const;
    virtual double color_rescale(const double&) const;
};

#endif /* SEARCHTREE_H_ */
