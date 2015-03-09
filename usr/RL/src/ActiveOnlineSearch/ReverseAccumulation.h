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
    enum TYPE { VALUES, FORWARD, REVERSE };

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
    void compute_values(std::vector<double> values);
    void forward_accumulation(std::vector<double> values, std::vector<double> differentials);
    void reverse_accumulation(std::vector<double> values, std::vector<double> differentials);
    void check_derivatives(std::vector<double> values, double epsilon = 1e-5, double delta = 1e-5);
    void plot_graph(const char* file_name) const;
    node_t add_node(QString node_label = "",
                    std::vector<QString> node_variables = std::vector<QString>(),
                    function_t node_function = [](std::vector<double>)->double{return NAN;});
    arc_t add_arc(node_t from,
                  node_t to,
                  function_t node_function = [](std::vector<double>)->double{return NAN;});
    std::vector<double> get_input_differentials() const;
    std::vector<double> get_output_differentials() const;
    std::vector<double> get_output_values() const;
private:
    void propagate_values(TYPE p);
    void assign_values(std::vector<double> values, TYPE a);
    double evaluate_node(const node_t & node, TYPE e);
};

#endif /* REVERSEACCUMULATION_H_ */
