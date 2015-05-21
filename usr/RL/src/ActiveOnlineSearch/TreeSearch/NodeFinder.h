#ifndef NODEFINDER_H_
#define NODEFINDER_H_

#include "SearchTree.h"

#include <deque>
#include <unordered_map>

namespace node_finder {

    typedef SearchTree::graph_t              graph_t;
    typedef SearchTree::node_t               node_t;
    typedef SearchTree::node_info_map_t      node_info_map_t;
    typedef SearchTree::action_handle_t      action_handle_t;
    typedef SearchTree::observation_handle_t observation_handle_t;
    typedef SearchTree::arc_node_t           arc_node_t;

    /**
     * Class for identifying existing nodes in a SearchTree. The different
     * implementations of this class correspond to different ways of building
     * the SearchTree. */
    class NodeFinder {
    public:
        NodeFinder() = default;
        virtual ~NodeFinder() = default;
        /**
         * Initialize the object with used graph and node info map. \warning
         * This function must be calld before calling any other functions. */
        virtual void init(const graph_t & graph,
                          const node_info_map_t & node_info_map) = 0;
        /**
         * Try to find the corresponding action node.
         * @param graph the search tree
         * @param observation_node the ancestor observation node
         * @param action the action to find a node for
         * @param node_info_map the map containing the NodeInfo */
        virtual arc_node_t find_action_node(const node_t & observation_node,
                                            const action_handle_t & action) = 0;
        /**
         * Try to find the corresponding observation node.
         * @param graph the search tree
         * @param action_node the ancestor action node
         * @param observation the observation to find a node for
         * @param node_info_map the map containing the NodeInfo */
        virtual arc_node_t find_observation_node(const node_t & action_node,
                                                 const observation_handle_t & observation) = 0;
        /**
         * Call back function to inform the class of a new action node.
         * @param graph the search tree
         * @param action_node the newly added action node
         * @param node_info_map the map containing the NodeInfo
         * \warning This function should be call \e after the node and all
         * connections have been added in the graph. */
        virtual void add_action_node(const node_t & action_node) = 0;
        /**
         * Call back function to inform the class of a new observation node.
         * @param graph the search tree
         * @param observation_node the newly added observation node
         * @param node_info_map the map containing the NodeInfo
         * \warning This function should be call \e after the node and all
         * connections have been added in the graph. */
        virtual void add_observation_node(const node_t & observation_node) = 0;
        /**
         * Call back function to inform the class of a action node to be erased.
         * @param graph the search tree
         * @param action_node the action node that will be erased
         * @param node_info_map the map containing the NodeInfo
         * \warning This function should be call \e before the node or any
         * connections were removed in the graph. */
        virtual void erase_action_node(const node_t & action_node) = 0;
        /**
         * Call back function to inform the class of a observation node to be
         * erased.
         * @param graph the search tree
         * @param observation_node the observation node that will be erased
         * @param node_info_map the map containing the NodeInfo
         * \warning This function should be call \e before the node or any
         * connections were removed in the graph. */
        virtual void erase_observation_node(const node_t & observation_node) = 0;
    };

    /**
     * Builds a full tree without reconnecting any nodes. */
    class PlainTree: public NodeFinder {
    protected:
        //----members----//
        const graph_t * graph = nullptr;
        const node_info_map_t * node_info_map = nullptr;
        //----methods----//
    public:
        virtual void init(const graph_t & g,
                          const node_info_map_t & m) override;
        virtual arc_node_t find_action_node(const node_t & observation_node,
                                            const action_handle_t & action) override;
        virtual arc_node_t find_observation_node(const node_t & action_node,
                                                 const observation_handle_t & observation) override;
        virtual void add_action_node(const node_t & action_node) override {}
        virtual void add_observation_node(const node_t & observation_node) override {}
        virtual void erase_action_node(const node_t & action_node) override {}
        virtual void erase_observation_node(const node_t & observation_node) override {}
    };

    /**
     * Modifies the PlainTree implementation. Identical observations that were
     * reached by different actions but from the same ancestor observation are
     * represented by the same observation node. Taking only observation nodes
     * into accout the resulting graph is again a tree. */
    class ObservationTree: public PlainTree {
    public:
        virtual arc_node_t find_observation_node(const node_t & action_node,
                                                 const observation_handle_t & observation) override;
    };

    /**
     * Modifies the ObservationTree implementation. Any identical observations
     * at the same depth of the SearchTree are represented by the same
     * node. That is, observations and the depth of the corresponding nodes are
     * tracked globally. */
    class FullDAG: public ObservationTree {
    public:
        //----typedefs----//
#ifdef UNIT_TESTS
        typedef char depth_t;
#else
        typedef int depth_t;
#endif
        //----members----//

        typedef std::unordered_map<
            observation_handle_t,
            node_t,
            AbstractEnvironment::ObservationHash,
            AbstractEnvironment::ObservationEq> observation_map_t;
        /**
         * List with maps of existing observations and the corresponding nodes. */
        std::deque<observation_map_t> observation_maps;
        /**
         * Map with depth of every node. #depth_offset must be subtracted to get
         * the true depth. */
        graph_t::NodeMap<depth_t> * depth_map = nullptr;
        /**
         * Offset of the #depth_map. Using an offset avoids decrementing the
         * depth for all nodes when the root node is erased. */
        depth_t depth_offset = 0;

        //----methods----//
    public:
        FullDAG() = default;
        virtual ~FullDAG();
        virtual void init(const graph_t & g,
                          const node_info_map_t & m) override;
        virtual depth_t get_depth(node_t node);
#ifdef UNIT_TESTS
        int get_true_depth(node_t node) {return(*depth_map)[node];}
#endif
        virtual void remove_depth_offset();
        virtual arc_node_t find_observation_node(const node_t & action_node,
                                                 const observation_handle_t & observation) override;
        virtual void add_observation_node(const node_t & observation_node) override;
        virtual void erase_observation_node(const node_t & observation_node) override;
    };

    /**
     * Modifies the ObservationTree implementation. Any identical observations
     *  are represented by the same node. In contrast to the FullDAG
     *  implementation this is done independently of their depth. */
    class FullGraph: public ObservationTree {
    public:
        //----members----//
        std::unordered_map<
            observation_handle_t,
            node_t,
            AbstractEnvironment::ObservationHash,
            AbstractEnvironment::ObservationEq> observation_map;

        //----methods----//
        virtual arc_node_t find_observation_node(const node_t & action_node,
                                                 const observation_handle_t & observation) override;
        virtual void add_observation_node(const node_t & observation_node) override;
        virtual void erase_observation_node(const node_t & observation_node) override;
    };

}; // end namespace node_finder

#endif /* NODEFINDER_H_ */
