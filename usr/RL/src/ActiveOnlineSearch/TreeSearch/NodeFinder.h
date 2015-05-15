#ifndef NODEFINDER_H_
#define NODEFINDER_H_

#include "SearchTree.h"

#include <deque>
#include <unordered_map>

#include <util/return_tuple.h>

#define DEBUG_LEVEL 2
#include <util/debug.h>

namespace node_finder {

    typedef SearchTree::graph_t              graph_t;
    typedef SearchTree::node_t               node_t;
    typedef SearchTree::arc_t                arc_t;
    typedef SearchTree::node_it_t            node_it_t;
    typedef SearchTree::arc_it_t             arc_it_t;
    typedef SearchTree::in_arc_it_t          in_arc_it_t;
    typedef SearchTree::out_arc_it_t         out_arc_it_t;
    typedef SearchTree::arc_node_t           arc_node_t;
    typedef SearchTree::NODE_TYPE            NODE_TYPE;
    typedef SearchTree::node_info_map_t      node_info_map_t;
    typedef SearchTree::action_handle_t      action_handle_t;
    typedef SearchTree::state_handle_t       state_handle_t;
    typedef SearchTree::observation_handle_t observation_handle_t;
    typedef SearchTree::reward_t             reward_t;

    using lemon::INVALID;

    /**
     * Class for identifying existing nodes in a SearchTree. The different
     * implementations of this class correspond to different ways of building
     * the SearchTree. */
    class NodeFinder {
    public:
        NodeFinder() = default;
        virtual ~NodeFinder() = default;
        /**
         * Try to find the corresponding action node.
         * @param graph the search tree
         * @param observation_node the ancestor observation node
         * @param action the action to find a node for
         * @param node_info_map the map containing the NodeInfo */
        virtual arc_node_t find_action_node(const graph_t & graph,
                                            const node_t & observation_node,
                                            const action_handle_t & action,
                                            const node_info_map_t & node_info_map) const = 0;
        /**
         * Try to find the corresponding observation node.
         * @param graph the search tree
         * @param action_node the ancestor action node
         * @param observation the observation to find a node for
         * @param node_info_map the map containing the NodeInfo */
        virtual arc_node_t find_observation_node(const graph_t & graph,
                                                 const node_t & action_node,
                                                 const observation_handle_t & observation,
                                                 const node_info_map_t & node_info_map) const = 0;
        /**
         * Call back function to inform the class of a new action node.
         * @param graph the search tree
         * @param action_node the newly added action node
         * @param node_info_map the map containing the NodeInfo */
        virtual void add_action_node(const graph_t & graph,
                                     const node_t & action_node,
                                     const node_info_map_t & node_info_map) = 0;
        /**
         * Call back function to inform the class of a new observation node.
         * @param graph the search tree
         * @param observation_node the newly added observation node
         * @param node_info_map the map containing the NodeInfo */
        virtual void add_observation_node(const graph_t & graph,
                                          const node_t & observation_node,
                                          const node_info_map_t & node_info_map) = 0;
        /**
         * Call back function to inform the class of a action node to be erased.
         * @param graph the search tree
         * @param action_node the action node that will be erased
         * @param node_info_map the map containing the NodeInfo */
        virtual void erase_action_node(const graph_t & graph,
                                       const node_t & action_node,
                                       const node_info_map_t & node_info_map) = 0;
        /**
         * Call back function to inform the class of a observation node to be
         * erased.
         * @param graph the search tree
         * @param observation_node the observation node that will be erased
         * @param node_info_map the map containing the NodeInfo */
        virtual void erase_observation_node(const graph_t & graph,
                                            const node_t & observation_node,
                                            const node_info_map_t & node_info_map) = 0;
    };

    /**
     * Builds a full tree without reconnecting any nodes. */
    class FullTree: public NodeFinder {
    public:
        virtual arc_node_t find_action_node(const graph_t & graph,
                                            const node_t & observation_node,
                                            const action_handle_t & action,
                                            const node_info_map_t & node_info_map) const override {
            DEBUG_EXPECT(0,node_info_map[observation_node].type==NODE_TYPE::OBSERVATION_NODE);
            for(out_arc_it_t arc(graph,observation_node); arc!=INVALID; ++arc) {
                node_t action_node = graph.target(arc);
                if(node_info_map[action_node].action==action) {
                    return arc_node_t(arc,action_node);
                }
            }
            return arc_node_t(INVALID,INVALID);
        }
        virtual arc_node_t find_observation_node(const graph_t & graph,
                                                 const node_t & action_node,
                                                 const observation_handle_t & observation,
                                                 const node_info_map_t & node_info_map) const override {
            DEBUG_OUT(1,"Find observation node: FullTree");
            DEBUG_EXPECT(0,node_info_map[action_node].type==NODE_TYPE::ACTION_NODE);
            for(out_arc_it_t arc(graph,action_node); arc!=INVALID; ++arc) {
                node_t observation_node = graph.target(arc);
                if(node_info_map[observation_node].observation==observation) {
                    return arc_node_t(arc,observation_node);
                }
            }
            return arc_node_t(INVALID,INVALID);
        }
        virtual void add_action_node(const graph_t & graph,
                                     const node_t & action_node,
                                     const node_info_map_t & node_info_map) override {}
        virtual void add_observation_node(const graph_t & graph,
                                          const node_t & observation_node,
                                          const node_info_map_t & node_info_map) override {}
        virtual void erase_action_node(const graph_t & graph,
                                       const node_t & action_node,
                                       const node_info_map_t & node_info_map) override {}
        virtual void erase_observation_node(const graph_t & graph,
                                            const node_t & observation_node,
                                            const node_info_map_t & node_info_map) override {}
    };

    /**
     * Modifies the FullTree implementation. Identical observations that were
     * reached by different actions but from the same ancestor observation are
     * represented by the same observation node. */
    class ObservationTree: public FullTree {
    public:
        virtual arc_node_t find_observation_node(const graph_t & graph,
                                                 const node_t & action_node,
                                                 const observation_handle_t & observation,
                                                 const node_info_map_t & node_info_map) const override {

            // first try "standard approach"
            {
                arc_t arc;
                node_t node;
                return_tuple::t(arc,node) = FullTree::find_observation_node(graph,action_node,observation,node_info_map);
                if(arc!=INVALID && node!=INVALID) {
                    DEBUG_OUT(2,"Found existing observation node (id=" << graph.id(node) << ") for action node (id=" << graph.id(action_node) << ")");
                    return arc_node_t(arc,node);
                }
                DEBUG_OUT(2,"Didn't find existing observation node for action node (id=" << graph.id(action_node) << ")");
            }

            DEBUG_OUT(1,"Find observation node: ObservationTree");
            DEBUG_EXPECT(0,node_info_map[action_node].type==NODE_TYPE::ACTION_NODE);

            for(in_arc_it_t arc_from_parent_observation_node(graph,action_node);
                arc_from_parent_observation_node!=INVALID;
                ++arc_from_parent_observation_node) {

                node_t parent_observation_node = graph.source(arc_from_parent_observation_node);

                for(out_arc_it_t arc_to_sibling_action_node(graph,parent_observation_node);
                    arc_to_sibling_action_node!=INVALID;
                    ++arc_to_sibling_action_node) {

                    node_t sibling_action_node = graph.target(arc_to_sibling_action_node);
                    // don't recheck (was in "standard approach" done above)
                    if(sibling_action_node==action_node) continue;

                    for(out_arc_it_t arc_to_observation_node(graph,sibling_action_node);
                        arc_to_observation_node!=INVALID;
                        ++arc_to_observation_node) {

                        node_t observation_node = graph.target(arc_to_observation_node);

                        if(*(node_info_map[observation_node].observation)==*(observation)) {
                            // return node but not arc, which comes from wrong action node
                            DEBUG_OUT(2,"Found existing observation node (id=" << graph.id(observation_node) << ") for action node (id=" << graph.id(action_node) << ")");
                            return arc_node_t(INVALID,observation_node);
                        } else {
                            DEBUG_OUT(2,"Didn't find existing observation node for action node (id=" << graph.id(action_node) << ")");
                        }
                    }
                }
            }
            return arc_node_t(INVALID,INVALID);
        }
    };

    /**
     * Modifies the FullTree implementation. Any identical observations at the
     * same depth of the SearchTree are represented by the same node. This makes
     * it necessary to globally keep track of observation nodes and their
     * depths. */
    class FullDAG: public FullTree {
    public:
        //----members----//
        typedef std::unordered_map<
            observation_handle_t,
            node_t,
            AbstractEnvironment::hash<observation_handle_t>> observation_map_t;
        std::deque<observation_map_t> observation_maps;
        #warning compute depth
        int depth = 0;

        //----methods----//
        virtual arc_node_t find_observation_node(const graph_t & graph,
                                                 const node_t & action_node,
                                                 const observation_handle_t & observation,
                                                 const node_info_map_t & node_info_map) const override {
            DEBUG_OUT(1,"Find observation node: FullDAG");
            DEBUG_EXPECT(0,node_info_map[action_node].type==NODE_TYPE::ACTION_NODE);
            if((int)observation_maps.size()>depth) {
                auto& map = observation_maps[depth];
                auto observation_node_it = map.find(observation);
                if(observation_node_it!=map.end()) {
                    node_t observation_node = observation_node_it->second;
                    for(in_arc_it_t arc(graph,observation_node); arc!=INVALID; ++arc) {
                        if(graph.source(arc)==action_node) {
                            return arc_node_t(arc,observation_node);
                        }
                    }
                    return arc_node_t(INVALID,observation_node);
                }
            }
            return arc_node_t(INVALID,INVALID);
        }
        virtual void add_observation_node(const graph_t & graph,
                                          const node_t & observation_node,
                                          const node_info_map_t & node_info_map) override {
            while((int)observation_maps.size()<=depth) observation_maps.push_back(observation_map_t());
            observation_maps[depth][node_info_map[observation_node].observation] = observation_node;
        }
        virtual void erase_observation_node(const graph_t & graph,
                                            const node_t & observation_node,
                                            const node_info_map_t & node_info_map) override {
            DEBUG_EXPECT(0,(int)observation_maps.size()>depth);
            int n = observation_maps[depth].erase(node_info_map[observation_node].observation);
            DEBUG_EXPECT(0,n==1);
            if(depth==0) { // root node will be erased
                DEBUG_EXPECT(0,observation_maps[0].size()==0);
                observation_maps.pop_front();
            }
        }
    };

}; // end namespace node_finder

#include <util/debug_exclude.h>

#endif /* NODEFINDER_H_ */
