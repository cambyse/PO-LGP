#ifndef REVERSEACCUMULATION_H_
#define REVERSEACCUMULATION_H_

#include <QString>

#include <lemon/list_graph.h>

#include <vector>
#include <functional>

class ReverseAccumulation {
    //----typedefs/classes----//
private:
    typedef lemon::ListDigraph graph_t;
    typedef graph_t::Node node_t;
    typedef graph_t::NodeIt node_it_t;
    typedef graph_t::Arc arc_t;
    typedef graph_t::ArcIt arc_it_t;
    typedef graph_t::OutArcIt out_arc_it_t;
    typedef graph_t::InArcIt in_arc_it_t;
    typedef std::function<double(std::vector<double>)> function_t;

    //----members----//
private:
    graph_t graph;
    graph_t::NodeMap<QString> node_labels;
    graph_t::NodeMap<double> node_values; // variable values
    graph_t::NodeMap<double> node_differentials;
    graph_t::ArcMap<double> arc_values;   // partial derivative values
    graph_t::NodeMap<std::vector<QString>> node_variables;
    graph_t::NodeMap<function_t> node_functions;
    graph_t::ArcMap<function_t> arc_functions;
    std::vector<node_t> input_nodes;
    std::vector<node_t> output_nodes;

    //----methods----//
public:
    ReverseAccumulation();
    virtual ~ReverseAccumulation() = default;
    void assign_values(std::vector<double> values);
    void propagate_values();
    void plot_graph(const char* file_name) const;
    node_t add_node(QString node_label = "",
                    std::vector<QString> node_variables = std::vector<QString>(),
                    function_t node_function = [](std::vector<double>)->double{return NAN;});
    arc_t add_arc(node_t from,
                  node_t to,
                  function_t node_function = [](std::vector<double>)->double{return NAN;});
private:
    double evaluate_node(const node_t & node);
};

#endif /* REVERSEACCUMULATION_H_ */
