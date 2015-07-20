#ifndef GRAPH_UTIL_H_
#define GRAPH_UTIL_H_

#include <vector>
#include <queue>
#include <stack>
#include <functional>
#include <unordered_set>

#include <util/graph_plotting.h>

#define DEBUG_LEVEL 0
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

    template<class graph_t, class map_graph_t = graph_t>
        class GraphFlooding {
    public:
        typedef typename graph_t::Node node_t;
        typedef typename graph_t::NodeIt node_it_t;
        typedef typename graph_t::OutArcIt out_arc_it_t;
        template<class T>
            using node_map_t = typename map_graph_t::template NodeMap<T>;

        const graph_t & graph;
        node_map_t<bool> & reached;
        std::stack<node_t> unprocessed_nodes;

    GraphFlooding(const graph_t & graph, node_map_t<bool> & reached):
        graph(graph),
            reached(reached)
            {}

        GraphFlooding & add_source(node_t node) {
            unprocessed_nodes.push(node);
            return *this;
        }

        void flood() {
            // init reached to false
            for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
                reached[node] = false;
            }
            // process
            while(!unprocessed_nodes.empty()) {
                node_t current_node = unprocessed_nodes.top();
                unprocessed_nodes.pop();
                for(out_arc_it_t arc(graph,current_node); arc!=lemon::INVALID; ++arc) {
                    node_t other_node = graph.target(arc);
                    if(!reached[other_node]) unprocessed_nodes.push(other_node);
                }
                reached[current_node] = true;
            }
        }
    };

    template<class graph_t>
        GraphFlooding<graph_t> graph_flooding(const graph_t & graph,
                                              typename graph_t::template NodeMap<bool> & reached) {
        return GraphFlooding<graph_t>(graph,reached);
    }

    /**
     * \example GraphPropagationExample.cpp This is an example of how to use the
     * GraphPropagation class. */

    /**
     * Class for propagating information through a graph. See
     * GraphPropagationExample.cpp for an example. */
    template<class GRAPH_1, class GRAPH_2 = GRAPH_1>
        class GraphPropagation {
        //----typedefs/classes----//
    public:
        typedef typename GRAPH_1::Node node_t;
        typedef typename GRAPH_1::NodeIt node_it_t;
        typedef typename GRAPH_1::Arc arc_t;
        typedef typename GRAPH_1::ArcIt arc_it_t;
        typedef typename GRAPH_1::OutArcIt out_arc_it_t;
        typedef typename GRAPH_1::InArcIt in_arc_it_t;
        template<class T>
            using node_map_t = typename GRAPH_2::template NodeMap<T>;
        typedef std::function<bool(node_t)> check_change_function_t;
        typedef std::unordered_set<node_t,NodeHashFunction<GRAPH_1>> node_set_t;

        //----members----//
    private:
        const GRAPH_1 & graph;
        node_set_t source_nodes;
        node_set_t unprocessed_nodes_set;
        std::deque<node_t> unprocessed_nodes_queue;
        bool local_reachable_map                        = false;
        bool local_processed_map                        = false;
        node_map_t<bool> * reachable_map                = nullptr;
        check_change_function_t * check_change_function = nullptr;
        node_map_t<bool> * processed_map                = nullptr;
        node_t next_node                                = lemon::INVALID;
        bool _allow_incomplete_updates                  = false;

        //----methods----//
    public:

        GraphPropagation() = delete;

        GraphPropagation(const GRAPH_1 & graph):
            graph(graph),
            source_nodes({},0,NodeHashFunction<GRAPH_1>(graph)),
            unprocessed_nodes_set({},0,NodeHashFunction<GRAPH_1>(graph))
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
            DEBUG_OUT(1,"Computing reachable nodes");
            find_reachable_nodes();
            // mark all nodes as unprocessed
            for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
                DEBUG_OUT(3,"Mark node " << graph.id(node) << " as unprocessed");
                (*processed_map)[node] = false;
            }
            // init set of unprocessed nodes
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
         * Whether to allow updates of nodes with unprocessed inputs. This is
         * important in loopy graphs. Incomplete updates will only be made if no
         * complete updates are possible. */
        GraphPropagation & allow_incomplete_updates(bool val) {
            _allow_incomplete_updates = val;
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
         * Returns true if the given node is reachable. \pre init() should be
         * run before using this function. */
        bool is_reachable(node_t node) const { return (*reachable_map)[node]; }

        /**
         * Returns true if the given node was already processed. */
        bool was_processed(node_t node) const { return (*processed_map)[node]; }

        /**
         * Returns the next node to be updated. \note The source nodes added
         * using add_source() are assumed to be up-to-date and are thus \e not
         * returned once more by the next() function. */
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
            node_t first_in_queue = unprocessed_nodes_queue.front();
            int iteration_counter = 0;
            while(!unprocessed_nodes_queue.empty()) {
                // increment counter
                ++iteration_counter;
                // assign new next node
                next_node = unprocessed_nodes_queue.front();
                unprocessed_nodes_queue.pop_front();
                unprocessed_nodes_set.erase(next_node);
                // check inputs
                bool inputs_available = true;
                DEBUG_OUT(2,"    checking inputs...");
                for(in_arc_it_t arc(graph,next_node); arc!=lemon::INVALID; ++arc) {
                    node_t input_node = graph.source(arc);
                    DEBUG_OUT(3,"    node " << graph.id(input_node) << " is " <<
                              (is_reachable(input_node)?"reachable":"not reachable") << " and " <<
                              (was_processed(input_node)?"was processed":"was not processed") );
                    if(is_reachable(input_node) && !was_processed(input_node)) {
                        DEBUG_OUT(3,"    node " << graph.id(input_node) << "is not available");
                        inputs_available = false;
                        break;
                    } else DEBUG_OUT(3,"    node " << graph.id(input_node) << " is available");
                }
                // break or continue?
                if(inputs_available) {
                    DEBUG_OUT(2,"        choosing node " << graph.id(next_node));
                    break;
                } else {
                    DEBUG_OUT(2,"        missing inputs for node " << graph.id(next_node));
                    if(next_node==first_in_queue && iteration_counter>1) {
                        if(_allow_incomplete_updates) {
                            DEBUG_OUT(2,"        choosing node (incomplete) " << graph.id(next_node));
                            break;
                        } else {
                            DEBUG_WARNING("Premature interruption because no updates are possible (consider allowing incomplete updates)");
                            next_node = lemon::INVALID;
                            break;
                        }
                    } else {
                        unprocessed_nodes_queue.push_back(next_node);
                        unprocessed_nodes_set.insert(next_node);
                        next_node = lemon::INVALID;
                    }
                    continue;
                }
            }
            return next_node;
        }

    private:

        /**
         * Floods the graph to determine all nodes that are reachable from the
         * source nodes. This function is called by init(). */
        GraphPropagation & find_reachable_nodes() {
            DEBUG_OUT(1,"Find reachable nodes by flooding...");
            auto flood = graph_flooding(graph,*reachable_map);
            for(node_t node : source_nodes) {
                DEBUG_OUT(2,"    add node " << graph.id(node) << " to source nodes");
                flood.add_source(node);
            }
            flood.flood();
            return *this;
        }

        bool changed(node_t node) const {
            if(check_change_function) return (*check_change_function)(node);
            else return !was_processed(node);
        }
    };

    template<class graph_t>
        GraphPropagation<graph_t,graph_t> GraphPropagationFactory(const graph_t & graph) {
        return GraphPropagation<graph_t,graph_t>(graph);
    }

} //end namespace graph_util

#include <util/debug_exclude.h>

#endif /* GRAPH_UTIL_H_ */
