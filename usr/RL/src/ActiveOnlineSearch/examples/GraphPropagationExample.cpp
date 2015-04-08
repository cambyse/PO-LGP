#include <iostream>
#include "../graph_util.h"
#include <util/util.h>
#include <util/graph_plotting.h>

typedef lemon::ListDigraph graph_t;
typedef graph_t::Node Node;
typedef graph_t::Arc Arc;
typedef graph_t::NodeIt NodeIt;
typedef graph_t::ArcIt ArcIt;
typedef graph_t::OutArcIt OutArcIt;
typedef graph_t::InArcIt InArcIt;
template<class T>
using NodeMap = graph_t::NodeMap<T>;

using std::cout;
using std::endl;

// updates the value of next_node by "diffusing" 10% of its predecessor values
void update(const graph_t & graph, const Node & next_node, NodeMap<double> & values);

int main(int argn, char ** args) {

    // ========================================================================
    // Construct a circular graph with one "external" node that is not part of
    // the circle. Then propagate a value through the graph.
    // ========================================================================

    // initialize graph and first two nodes
    graph_t graph;
    Node external_node = graph.addNode();
    Node initial_node = graph.addNode();
    graph.addArc(external_node,initial_node);

    // construct circle
    Node previous_node = initial_node;
    Node next_node = initial_node;
    for(int i=0; i<10; ++i) {
        previous_node = next_node;
        next_node = graph.addNode();
        graph.addArc(previous_node,next_node);
    }

    // close circle
    graph.addArc(next_node,initial_node);

    // initialize all values with zero and external_node and initial_node with
    // one
    NodeMap<double> values(graph,0);
    values[external_node] = 1;
    values[initial_node] = 1;

    // get the GraphPropagation object (use factory automatically deduce graph
    // type)
    auto graph_propagation = graph_util::GraphPropagationFactory(graph);

    // add initial_node as source and initialize internal data structures
    graph_propagation.add_source(initial_node).init();

    // check: all nodes except for the external node are reachable
    for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
        if(node==external_node && graph_propagation.is_reachable(node)) {
            cout << "ERROR: External node should not be reachable" << endl;
        } else if(node!=external_node && !graph_propagation.is_reachable(node)) {
            cout << "ERROR: Node with id " << graph.id(node) << " should be reachable" << endl;
        }
    }

    // run propagation with default check-change function
    for(Node next_node=graph_propagation.next();
        next_node!=lemon::INVALID;
        next_node=graph_propagation.next()) {
        update(graph, next_node, values);
    }

    // check: all nodes have a value greater than zero but smaller than one
    // (decreasing along the circle) except for the external node, which has a
    // value of 1
    for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
        if(node==external_node && values[node]!=1) {
            cout << "ERROR: External node has wrong value (" << values[node] << ")" << endl;
        } else if(node!=external_node && (values[node]>=1 || values[node]<=0)) {
            cout << "ERROR: Node with id " << graph.id(node) << " has wrong value (" << values[node] << ")" << endl;
        }
    }

    // change check-change function to monitor the value and re-initialize
    graph_util::GraphPropagation<graph_t>::check_change_function_t check_change_function = [&](Node node)->bool{
        static NodeMap<double> old_values(graph,0);
        double return_value = fabs(old_values[node]-values[node])>1e-10;
        old_values[node] = values[node];
        return return_value;
    };
    graph_propagation.set_check_change_function(check_change_function).init();

    // run propagation with new check-change function
    for(Node next_node=graph_propagation.next();
        next_node!=lemon::INVALID;
        next_node=graph_propagation.next()) {
        update(graph, next_node, values);
    }

    // check: all nodes have a value close to 1
    for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
        if(fabs(values[node]-1)>1e-5) {
            cout << "ERROR: node has wrong value (" << values[node] << ")" << endl;
        }
    }
    return 0;
}

void update(const graph_t & graph, const Node & next_node, NodeMap<double> & values) {
    // 10% diffuses from predecessors
    double val = 0;
    int pred = 0;
    for(InArcIt arc(graph,next_node); arc!=lemon::INVALID; ++arc) {
        val += values[graph.source(arc)];
        ++pred;
    }
    val /= pred;
    values[next_node] = 0.9*values[next_node] + 0.1*val;
}
