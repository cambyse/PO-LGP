#ifndef GRAPH_UTIL_H_
#define GRAPH_UTIL_H_

#include <vector>
#include <queue>
#include <functional>
#include <unordered_set>

#include <lemon/list_graph.h>
#include <lemon/maps.h>
#include <lemon/connectivity.h>
#include <lemon/dfs.h>
#include <lemon/adaptors.h>
#include <lemon/concepts/digraph.h>

#define DEBUG_LEVEL 3
#include <util/debug.h>

namespace graph_util {


    /**
     * Computes a hash for nodes. This class returns the node ID given the used
     * graph. The default constructor is deleted because instanciating an object
     * without giving a specific graph does not make sense. */
    template<class graph_t>
    struct NodeHashFunction {
    public:
        typedef typename graph_t::Node node_t;
    public:
        NodeHashFunction() = delete;
        NodeHashFunction(const graph_t & graph): graph(graph) {}
        int operator()(const node_t & node)const{return graph.id(node);}
    private:
        const graph_t & graph;
    };

    /**
     * \example GraphPropagationExample.cpp This is an example of how to use the
     * GraphPropagation class. */

    /**
     * Class for propagating information through a graph. See
     * GraphPropagationExample.cpp for an example. */
    template<class graph_t>
        class GraphPropagation {
        //----typedefs/classes----//
    public:
        typedef typename graph_t::Node node_t;
        typedef typename graph_t::NodeIt node_it_t;
        typedef typename graph_t::Arc arc_t;
        typedef typename graph_t::ArcIt arc_it_t;
        typedef typename graph_t::OutArcIt out_arc_it_t;
        typedef typename graph_t::InArcIt in_arc_it_t;
        template<class T>
            using node_map_t = typename graph_t::template NodeMap<T>;
        typedef std::function<bool(node_t)> check_change_function_t;
        typedef std::unordered_set<node_t,NodeHashFunction<graph_t>> node_set_t;

        //----members----//
    private:
        const graph_t & graph;
        node_set_t source_nodes;
        node_set_t unprocessed_nodes_set;
        std::deque<node_t> unprocessed_nodes_queue;
        bool local_reachable_map         = false;
        bool local_processed_map         = false;
        bool reachable_nodes_computed    = false;
        node_map_t<bool> * reachable_map                = nullptr;
        check_change_function_t * check_change_function = nullptr;
        node_map_t<bool> * processed_map                = nullptr;
        node_t next_node = lemon::INVALID;

        //----methods----//
    public:

        GraphPropagation() = delete;

        GraphPropagation(const graph_t & graph):
            graph(graph),
            source_nodes({},0,NodeHashFunction<graph_t>(graph)),
            unprocessed_nodes_set({},0,NodeHashFunction<graph_t>(graph))
        {}

        virtual ~GraphPropagation() {
            if(local_reachable_map) delete reachable_map;
            if(local_processed_map) delete processed_map;
        }

        /**
         * Initialize all members that were not set explicitly. */
        GraphPropagation & init() {
            // init reachable map
            if(!reachable_map) {
                DEBUG_OUT(1,"Using local reachable map");
                reachable_map = new node_map_t<bool>(graph);
                local_reachable_map = true;
            } else DEBUG_OUT(1,"Using external reachable map");
            // init processed map
            if(!processed_map) {
                DEBUG_OUT(1,"Using local processed map");
                processed_map = new node_map_t<bool>(graph,false);
                local_processed_map = true;
            } else DEBUG_OUT(1,"Using external processed map");
            // compute reachable nodes
            if(!reachable_nodes_computed) {
                DEBUG_OUT(1,"Computing reachable nodes");
                find_reachable_nodes();
            } else DEBUG_OUT(1,"Reachable already computed");
            // mark all nodes as unprocessed
            for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
                DEBUG_OUT(3,"Mark node " << graph.id(node) << " as unprocessed");
                (*processed_map)[node] = false;
            }
            // init set of unprocessed nodes and mark source nodes as processed
            {
                // clear (in case it's not empty e.g. from previous runs)
                unprocessed_nodes_set.clear();
                std::deque<node_t> empty;
                unprocessed_nodes_queue.swap(empty);
                // add successors of source nodes
                for(node_t source_node : source_nodes) {
                    for(out_arc_it_t arc(graph,source_node); arc!=lemon::INVALID; ++arc) {
                        node_t other_node = graph.target(arc);
                        if(unprocessed_nodes_set.find(other_node)==unprocessed_nodes_set.end()) {
                            unprocessed_nodes_set.insert(other_node);
                            unprocessed_nodes_queue.push_front(other_node);
                        }
                    }
                    // mark source nodes as processed
                    (*processed_map)[source_node] = true;
                    DEBUG_OUT(3,"Mark node " << graph.id(source_node) << " as processed");
                }
            }
            // next node is invalid initially
            next_node = lemon::INVALID;
            return *this;
        }

        /**
         * Set check-change function. */
        GraphPropagation & set_check_change_function(check_change_function_t & function) {
            check_change_function = &function;
            return *this;
        }

        /**
         * Sets the map to be used for storing whether a node can be reached. */
        GraphPropagation & set_reachable_map(node_map_t<bool> & map) {
            if(local_reachable_map) {
                delete reachable_map;
                local_reachable_map = false;
            }
            reachable_map = &map;
            return *this;
        }

        /**
         * Sets the map to be used for storing whether a node was already
         * processed. */
        GraphPropagation & set_processed_map(node_map_t<bool> & map) {
            if(local_processed_map) {
                delete processed_map;
                local_processed_map = false;
            }
            processed_map = &map;
            return *this;
        }

        /**
         * Adds \p node to the set of source nodes from which the propagation is
         started. */
        GraphPropagation & add_source(node_t node) {
            source_nodes.insert(node);
            return *this;
        }

        /**
         * Performs a depth-first search to determine all nodes reachable from
         * the source nodes. If not called explicitly, this function is called
         * by init(). \warning If you add source nodes using add_source() after
         * calling find_reachable_nodes() init() will not update the reachable
         * nodes. */
        GraphPropagation & find_reachable_nodes() {
            lemon::Dfs<graph_t> search(graph);
            search.init();
            for(node_t node : source_nodes) {
                search.addSource(node);
            }
            search.reachedMap(*reachable_map).start();
            reachable_nodes_computed = true;
            return *this;
        }

        /**
         * Returns true if the given node is reachable.  \pre
         * find_reachable_nodes() or init() should be run before using this
         * function. */
        bool is_reachable(node_t node) const { return (*reachable_map)[node]; }

        /**
         * Returns true if the given node was already processed. */
        bool was_processed(node_t node) const { return (*processed_map)[node]; }

        /**
         * Returns the next node to be updated. */
        node_t next() {
            DEBUG_OUT(1,"Find next node...");
            // check for changes
            DEBUG_OUT(2,"    check if old next_node is valid and changed");
            if(next_node!=lemon::INVALID) {
                // add successor nodes to unprocessed (only if node changed)
                if(changed(next_node)) {
                    for(out_arc_it_t arc(graph,next_node); arc!=lemon::INVALID; ++arc) {
                        node_t node = graph.target(arc);
                        if(unprocessed_nodes_set.find(node)==unprocessed_nodes_set.end()) {
                            unprocessed_nodes_set.insert(node);
                            unprocessed_nodes_queue.push_front(node);
                        };
                    }
                }
                // mark as processed
                (*processed_map)[next_node] = true;
            }
            // check unprocessed nodes until finding one with all inputs
            // available
            IF_DEBUG(2) {
                DEBUG_OUT(2,"    unprocessed nodes: ");
                for(node_t node : unprocessed_nodes_set) DEBUG_OUT(2,"        " << graph.id(node));
            }
            next_node = lemon::INVALID;
            while(!unprocessed_nodes_queue.empty()) {
                // assign new next node
                next_node = unprocessed_nodes_queue.front();
                unprocessed_nodes_queue.pop_front();
                unprocessed_nodes_set.erase(next_node);
                // check inputs
                bool inputs_available = true;
                DEBUG_OUT(2,"    checking inputs...");
                for(in_arc_it_t arc(graph,next_node); arc!=lemon::INVALID; ++arc) {
                    node_t source_node = graph.source(arc);
                    if(is_reachable(source_node) && !was_processed(source_node)) {
                        DEBUG_OUT(3,"    node " << graph.id(source_node) << " not available");
                        inputs_available = false;
                        break;
                    } else DEBUG_OUT(3,"    node " << graph.id(source_node) << " is available");
                }
                // break or continue?
                if(inputs_available) {
                    DEBUG_OUT(2,"        choosing node " << graph.id(next_node));
                    break;
                } else {
                    DEBUG_OUT(2,"        missing inputs for node " << graph.id(next_node));
                    next_node = lemon::INVALID;
                    continue;
                }
            }
            return next_node;
        }

        private:
            bool changed(node_t node) const {
                if(check_change_function) return (*check_change_function)(node);
                else return !was_processed(node);
            }
    };

    template<class graph_t>
        GraphPropagation<graph_t> GraphPropagationFactory(const graph_t & graph) {
        return GraphPropagation<graph_t>(graph);
    }

} //end namespace graph_util

#include <util/debug_exclude.h>

#endif /* GRAPH_UTIL_H_ */
