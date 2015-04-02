#include "ComputationalGraph.h"

typedef ComputationalGraph CG;

CG::ComputationalGraph():
    ComputationalConstGraph(),
    graph(this->dummy_graph)
{}

CG::ComputationalGraph(graph_t & graph):
    ComputationalConstGraph(graph),
    graph(graph)
{}

CG::node_t CG::add_node(QString label,
                        std::vector<QString> variables,
                        function_t function) {
    node_t node = graph.addNode();
    set_node(node,label,variables,function);
    return node;
}

CG::arc_t CG::add_arc(node_t from,
                      node_t to,
                      function_t function) {
    arc_t arc = graph.addArc(from, to);
    set_arc(arc,function);
    return arc;
}
