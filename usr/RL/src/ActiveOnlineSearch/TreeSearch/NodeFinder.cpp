#include "NodeFinder.h"

#include <limits.h>

#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
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
    typedef SearchTree::observation_handle_t observation_handle_t;
    typedef SearchTree::reward_t             reward_t;

    using lemon::INVALID;

    void PlainTree::init(const graph_t & g,
                         const node_info_map_t & m) {
        graph = &g;
        node_info_map = &m;
    }
    arc_node_t PlainTree::find_action_node(const node_t & observation_node,
                                          const action_handle_t & action) {
        DEBUG_EXPECT(0,(*node_info_map)[observation_node].type==NODE_TYPE::OBSERVATION_NODE);
        for(out_arc_it_t arc(*graph,observation_node); arc!=INVALID; ++arc) {
            node_t action_node = graph->target(arc);
            if(*((*node_info_map)[action_node].action)==*action) {
                return arc_node_t(arc,action_node,false,false);
            }
        }
        return arc_node_t(INVALID,INVALID,false,false);
    }
    arc_node_t PlainTree::find_observation_node(const node_t & action_node,
                                               const observation_handle_t & observation) {
        DEBUG_OUT(1,"Find observation node: PlainTree");
        DEBUG_EXPECT(0,(*node_info_map)[action_node].type==NODE_TYPE::ACTION_NODE);
        for(out_arc_it_t arc(*graph,action_node); arc!=INVALID; ++arc) {
            node_t observation_node = graph->target(arc);
            if(*((*node_info_map)[observation_node].observation)==*observation) {
                return arc_node_t(arc,observation_node,false,false);
            }
        }
        return arc_node_t(INVALID,INVALID,false,false);
    }


    arc_node_t ObservationTree::find_observation_node(const node_t & action_node,
                                                      const observation_handle_t & observation) {

        // first try "standard approach" of PlainTree
        {
            arc_t arc;
            node_t node;
            bool new_arc, new_node;
            return_tuple::t(arc,node,new_arc,new_node) = PlainTree::find_observation_node(action_node,observation);
            if(node!=INVALID) {
                return arc_node_t(arc,node,new_arc,new_node);
            }
        }

        DEBUG_OUT(1,"Find observation node: ObservationTree");
        DEBUG_EXPECT(0,(*node_info_map)[action_node].type==NODE_TYPE::ACTION_NODE);

        for(in_arc_it_t arc_from_parent_observation_node(*graph,action_node);
            arc_from_parent_observation_node!=INVALID;
            ++arc_from_parent_observation_node) {

            node_t parent_observation_node = graph->source(arc_from_parent_observation_node);

            for(out_arc_it_t arc_to_sibling_action_node(*graph,parent_observation_node);
                arc_to_sibling_action_node!=INVALID;
                ++arc_to_sibling_action_node) {

                node_t sibling_action_node = graph->target(arc_to_sibling_action_node);
                // don't recheck (was in "standard approach" done above)
                if(sibling_action_node==action_node) continue;

                for(out_arc_it_t arc_to_observation_node(*graph,sibling_action_node);
                    arc_to_observation_node!=INVALID;
                    ++arc_to_observation_node) {

                    node_t observation_node = graph->target(arc_to_observation_node);

                    if(*((*node_info_map)[observation_node].observation)==*(observation)) {
                        // return node but not arc, which comes from wrong action node
                        return arc_node_t(INVALID,observation_node,false,false);
                    }
                }
            }
        }
        return arc_node_t(INVALID,INVALID,false,false);
    }
    FullDAG::~FullDAG() {
        delete depth_map;
    }
    void FullDAG::init(const graph_t & g,
                       const node_info_map_t & m) {
        ObservationTree::init(g,m);
        observation_maps.clear();
        delete depth_map;
        depth_map = new graph_t::NodeMap<depth_t>(g);
        depth_offset = 0;
    }
    FullDAG::depth_t FullDAG::get_depth(node_t node) {
        DEBUG_OUT(1,"depth=" << (int)((*depth_map)[node] - depth_offset));
        DEBUG_OUT(1,"depth_offset=" << (int)depth_offset);
        return (*depth_map)[node] - depth_offset;
    }
    void FullDAG::remove_depth_offset() {
        for(node_it_t node(*graph); node!=INVALID; ++node) {
            depth_t new_depth = (*depth_map)[node] - depth_offset;
            (*depth_map)[node] = new_depth;
        }
        depth_offset = 0;
#ifdef UNIT_TESTS
        DEBUG_OUT(0,"UNIT TESTS: remove depth offset");
#endif
    }
    arc_node_t FullDAG::find_observation_node(const node_t & action_node,
                                              const observation_handle_t & observation) {
        // first try "standard approach" of ObservationTree
        {
            arc_t arc;
            node_t node;
            bool new_arc, new_node;
            return_tuple::t(arc,node,new_arc,new_node) = ObservationTree::find_observation_node(action_node,observation);
            if(node!=INVALID) {
                return arc_node_t(arc,node,new_arc,new_node);
            }
        }

        // look up in observation map
        DEBUG_EXPECT(0,(*node_info_map)[action_node].type==NODE_TYPE::ACTION_NODE);
        depth_t depth = get_depth(graph->source(in_arc_it_t(*graph,action_node)))+1;
        DEBUG_OUT(1,"Find observation node: FullDAG");
        if((depth_t)observation_maps.size()>depth) {
            auto& map = observation_maps[depth];
            auto observation_node_it = map.find(observation);
            if(observation_node_it!=map.end()) {
                return arc_node_t(INVALID,observation_node_it->second,false,false);
            }
        }
        return arc_node_t(INVALID,INVALID,false,false);
    }
    void FullDAG::add_observation_node(const node_t & observation_node) {
        // get depth from ancestor observation node and set in map
        depth_t depth;
        in_arc_it_t arc(*graph,observation_node);
        if(arc==INVALID) {
            // root node
            depth = 0;
        } else {
            node_t action_node = graph->source(arc);
            arc = in_arc_it_t(*graph,action_node);
            DEBUG_EXPECT(0,arc!=INVALID);
            depth = get_depth(graph->source(arc))+1;
        }
        (*depth_map)[observation_node] = depth + depth_offset;
        if(depth+depth_offset>std::numeric_limits<depth_t>::max()/2) {
            remove_depth_offset();
        }
        // add observation to corresponding map (extend list if necessary)
        while((depth_t)observation_maps.size()<=depth) observation_maps.push_back(observation_map_t());
        observation_handle_t observation = (*node_info_map)[observation_node].observation;
        if(observation!=nullptr) {
            observation_maps[depth][observation] = observation_node;
            DEBUG_OUT(1,"Added observation node (id=" << graph->id(observation_node)
                      << ") at depth " << depth);
        } else {
            DEBUG_OUT(1,"Got null-pointer observation -- must be root node.");
        }
    }
    void FullDAG::erase_observation_node(const node_t & observation_node) {
        depth_t depth = get_depth(observation_node);
        DEBUG_EXPECT(0,(depth_t)observation_maps.size()>depth);
        observation_handle_t observation = (*node_info_map)[observation_node].observation;
        if(observation!=nullptr) {
            int n = observation_maps[depth].erase(observation);
            DEBUG_EXPECT(0,n==1);
        }
        if(depth==0 && observation_maps[0].size()==0) {
            // first map is empty so we can erase it
            observation_maps.pop_front();
            ++depth_offset;
        }
        DEBUG_OUT(1,"Erased observation node (id=" << graph->id(observation_node)
                  << ") at depth " << depth);
    }

    void FullGraph::init(const graph_t & g,
                         const node_info_map_t & m) {
        ObservationTree::init(g,m);
        observation_map.clear();
    }

    arc_node_t FullGraph::find_observation_node(const node_t & action_node,
                                                const observation_handle_t & observation) {
        // first try "standard approach" of ObservationTree
        {
            arc_t arc;
            node_t node;
            bool new_arc, new_node;
            return_tuple::t(arc,node,new_arc,new_node) = ObservationTree::find_observation_node(action_node,observation);
            if(node!=INVALID) {
                return arc_node_t(arc,node,new_arc,new_node);
            }
        }

        // look up in observation map
        DEBUG_OUT(1,"Find observation node: FullGraph");
        DEBUG_EXPECT(0,(*node_info_map)[action_node].type==NODE_TYPE::ACTION_NODE);
        auto observation_node_it = observation_map.find(observation);
        if(observation_node_it!=observation_map.end()) {
            return arc_node_t(INVALID,observation_node_it->second,false,false);
        }
        return arc_node_t(INVALID,INVALID,false,false);
    }
    void FullGraph::add_observation_node(const node_t & observation_node) {
        observation_handle_t observation = (*node_info_map)[observation_node].observation;
        if(observation!=nullptr) {
            observation_map[observation] = observation_node;
            DEBUG_OUT(1,"Added observation node (id=" << graph->id(observation_node) << ")");
        }
    }
    void FullGraph::erase_observation_node(const node_t & observation_node) {
        observation_handle_t observation = (*node_info_map)[observation_node].observation;
        if(observation!=nullptr) {
            int n = observation_map.erase(observation);
            DEBUG_EXPECT(0,n==1);
        }
        DEBUG_OUT(1,"Erased observation node (id=" << graph->id(observation_node) << ")");
    }

}; // end namespace node_finder
