#ifndef GRAPH_UTIL_H_
#define GRAPH_UTIL_H_

#include <vector>
#include <functional>
#include <unordered_set>

#include <lemon/list_graph.h>
#include <lemon/maps.h>
#include <lemon/connectivity.h>
#include <lemon/dfs.h>
#include <lemon/adaptors.h>
#include <lemon/concepts/digraph.h>

#ifdef UNIT_TESTS
#define DEBUG_LEVEL 1
#else
#define DEBUG_LEVEL 0
#endif
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
     * Class for propagating information through a graph. */
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
        typedef std::vector<node_t> source_node_queue_t;
        typedef std::function<bool(node_t)> check_change_function_t;
        typedef std::unordered_set<node_t,NodeHashFunction<graph_t>> node_set_t;

        //----members----//
    private:
        const graph_t & graph;
        source_node_queue_t source_node_queue;
        node_set_t unprocessed_nodes;
        node_set_t pending_nodes;
        bool local_reachable_map = false;
        node_map_t<bool> * reachable_map = nullptr;
        bool local_check_change_function = false;
        check_change_function_t * check_change_function = nullptr;
        bool reachable_nodes_computed = false;
        bool local_processed_map = false;
        node_map_t<bool> * processed_map = nullptr;
        node_t next_node = lemon::INVALID;

        //----methods----//
    public:

        GraphPropagation() = delete;

        GraphPropagation(const graph_t & graph):
            graph(graph),
            unprocessed_nodes({},0,NodeHashFunction<graph_t>(graph)),
            pending_nodes({},0,NodeHashFunction<graph_t>(graph))
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
                DEBUG_OUT(2,"Using local reachable map");
                reachable_map = new node_map_t<bool>(graph);
                local_reachable_map = true;
            } else {
                DEBUG_OUT(2,"Using external reachable map");
            }
            // init processed map
            if(!processed_map) {
                DEBUG_OUT(2,"Using local processed map");
                processed_map = new node_map_t<bool>(graph,false);
                local_processed_map = true;
            } else {
                DEBUG_OUT(2,"Using external processed map");
            }
            // init check-change function
            if(!check_change_function) {
                DEBUG_OUT(2,"Using default check-change function");
                auto default_check_change_function = [&](node_t node)->bool {
                    static node_map_t<bool> changed(graph,true);
                    bool return_value = changed[node];
                    changed[node] = false;
                    return return_value;
                };
                check_change_function = new check_change_function_t(default_check_change_function);
                local_check_change_function = true;
            } else {
                DEBUG_OUT(2,"Using custom check-change function");
            }
            // compute reachable nodes
            if(!reachable_nodes_computed) {
                DEBUG_OUT(2,"Computing reachable nodes");
                find_reachable_nodes();
            } else {
                DEBUG_OUT(2,"Reachable already computed");
            }
            // init set of unprocessed nodes and mark source nodes as processed
            for(node_t node : source_node_queue) {
                for(out_arc_it_t arc(graph,node); arc!=lemon::INVALID; ++arc) {
                    unprocessed_nodes.insert(graph.target(arc));
                }
                (*processed_map)[node] = true;
            }
            return *this;
        }

        /**
         * Set check-change function. */
        GraphPropagation & set_check_change_function(check_change_function_t & function) {
            if(local_check_change_function) {
                delete check_change_function;
                local_check_change_function = false;
            }
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
            source_node_queue.push_back(node);
            return *this;
        }

        /**
         * Performs a depth-first search to determine all nodes reachable from
         * the source nodes. If not called explicitly, this function is called
         * by init(). \warning If you add source nodes using add_source() after
         * calling this function init() will not update the reachable nodes. */
        GraphPropagation & find_reachable_nodes() {
            lemon::Dfs<graph_t> search(graph);
            search.init();
            for(node_t node : source_node_queue) {
                search.addSource(node);
            }
            search.reachedMap(*reachable_map).start();
            reachable_nodes_computed = true;
            return *this;
        }

        /**
         * Returns true if the given node is reachable.  \pre
         * find_reachable_nodes() should be run before using this function. */
        bool is_reachable(node_t node) const { return (*reachable_map)[node]; }

        /**
         * Returns true if the given node was already processed. */
        bool was_processed(node_t node) const { return (*processed_map)[node]; }

        /**
         * Returns the next node to be updated. */
        node_t next() {
            DEBUG_OUT(2,"Find next node...");
            // check for changes
            DEBUG_OUT(3,"    check if old next_node is valid and changed");
            if(next_node!=lemon::INVALID) {
                // add successor nodes to unprocessed (only if node changed)
                if((*check_change_function)(next_node)) {
                    for(out_arc_it_t arc(graph,next_node); arc!=lemon::INVALID; ++arc) {
                        unprocessed_nodes.insert(graph.target(arc));
                    }
                }
                // mark as processed
                (*processed_map)[next_node] = true;
            }
            // check unprocessed nodes until finding one with all inputs
            // available
            DEBUG_OUT(3,"    unprocessed nodes: ");
            IF_DEBUG(2) {
                for(node_t node : unprocessed_nodes) {
                    DEBUG_OUT(3,"        " << graph.id(node));
                }
            }
            auto next_node_it = unprocessed_nodes.begin();
            while(next_node_it!=unprocessed_nodes.end()) {
                next_node = *next_node_it;
                bool inputs_available = true;
                for(in_arc_it_t arc(graph,next_node); arc!=lemon::INVALID; ++arc) {
                    node_t source_node = graph.source(arc);
                    if(is_reachable(source_node) && !was_processed(source_node)) {
                        inputs_available = false;
                        break;
                    }
                }
                // forward iterator (in case of continuing the search)
                ++next_node_it;
                // erase node from unprocessed
                unprocessed_nodes.erase(next_node);
                IF_DEBUG(1) {
                    // add to pending nodes if NOT all inputs are available
                    if(!inputs_available) pending_nodes.erase(next_node);
                }
                // return/continue
                if(inputs_available) {
                    IF_DEBUG(1) {
                        // remove from pending nodes
                        pending_nodes.erase(next_node);
                    }
                    // return
                    return next_node;
                } else {
                    // continue seaching
                    continue;
                }
            }
            next_node = lemon::INVALID;
            return next_node;
        }

        bool check_pending_nodes() const {
            if(DEBUG_LEVEL<=0) {
                DEBUG_ERROR("For this function to work properly recompile with DEBUG_LEVEL greater than zero.");
            }
            IF_DEBUG(2) {
                DEBUG_OUT(0,"Pending nodes:");
                for(node_t node : pending_nodes) {
                    DEBUG_OUT(0,"    " << graph.id(node));
                }
            }
            return pending_nodes.size()==0;
        }
    };

} //end namespace graph_util

#include <util/debug_exclude.h>

#endif /* GRAPH_UTIL_H_ */
