#ifndef COMPUTATIONAL_GRAPH_H_
#define COMPUTATIONAL_GRAPH_H_

#include "ComputationalConstGraph.h"

class ComputationalGraph: public ComputationalConstGraph {
    //----typedefs/classes----//

    //----members----//
private:
    /**
     * The graph-object. This is a reference to the graph object given in the
     * constructor. */
    graph_t & graph;

    //----methods----//
public:
    ComputationalGraph();
    ComputationalGraph(graph_t & graph);
    virtual ~ComputationalGraph() = default;
    /**
     * Add and set a node/variable.
     *
     * @param node_label The variable name used as ID for function calls of
     * dependend variables.
     *
     * @param node_variables List of node/variable labels used to construct the
     * array given to \e node_function as argument.
     *
     * @param node_function The function for computing the node/variable
     * value. */
    node_t add_node(QString node_label = "",
                    std::vector<QString> node_variables = std::vector<QString>(),
                    function_t node_function = [](std::vector<double>)->double{return NAN;});
    /**
     * Add and set an arc.
     *
     * @param from Node/variable that serves as input.
     *
     * @param to Node/variable whose value depends on the one given by \e
     * from.
     *
     * @param node_function The function to compute the partial derivative of \e
     * to w.r.t. \e from. */
    arc_t add_arc(node_t from,
                  node_t to,
                  function_t node_function = [](std::vector<double>)->double{return NAN;});
};

#endif /* COMPUTATIONAL_GRAPH_H_ */
