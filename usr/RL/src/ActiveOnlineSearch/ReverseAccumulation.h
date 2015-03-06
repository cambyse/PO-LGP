#ifndef REVERSEACCUMULATION_H_
#define REVERSEACCUMULATION_H_

#include <QString>

#include <lemon/list_graph.h>

#include <vector>
#include <functional>

class ReverseAccumulation {
    //----typedefs/classes----//
    typedef lemon::ListDigraph graph_t;
    typedef graph_t::Node node_t;
    typedef graph_t::NodeIt node_it_t;
    typedef graph_t::Arc arc_t;
    typedef graph_t::ArcIt arc_it_t;
    typedef graph_t::OutArcIt out_arc_it_t;
    typedef graph_t::InArcIt in_arc_it_t;
    typedef std::function<double(std::vector<double>)> node_function_t;

    //----members----//
    graph_t graph;
    graph_t::NodeMap<QString> node_labels;
    graph_t::NodeMap<double> node_values;
    graph_t::NodeMap<std::pair<std::vector<QString>, node_function_t>> node_functions;
    std::vector<node_t> input_nodes;
    std::vector<node_t> output_nodes;

    //----methods----//
public:
    ReverseAccumulation();
    virtual ~ReverseAccumulation() = default;
    void assign_values(std::vector<double> values);
    void propagate_values();
    void plot_graph(const char* file_name) const;
private:
    double map_function(const node_t & node,
                        const std::pair<std::vector<QString>, node_function_t> & func) const;
};

#endif /* REVERSEACCUMULATION_H_ */
