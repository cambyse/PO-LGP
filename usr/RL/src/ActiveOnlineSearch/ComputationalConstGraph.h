#ifndef COMPUTATIONALCONSTGRAPH_H_
#define COMPUTATIONALCONSTGRAPH_H_

#include <QString>

#include <lemon/list_graph.h>

#include <vector>
#include <functional>

/** A class for constructing and using a computational graph for computing
 * function values and derivatives.
 *
 * The unit tests unit_tests.cpp implement the following graph
 *
 * \htmlonly <img src="graph.jpg" width="20%"/> \endhtmlonly
 * */
class ComputationalConstGraph {
    //----typedefs/classes----//
public:
    /**
     * Type of the graph that is being used. */
    typedef lemon::ListDigraph graph_t;
    /**
     * Type of nodes in the graph. */
    typedef graph_t::Node node_t;
    /**
     * Type of iterators over nodes in the graph. */
    typedef graph_t::NodeIt node_it_t;
    /**
     * Type of arcs in the graph. */
    typedef graph_t::Arc arc_t;
    /**
     * Type of iterators over arcs (incoming and outgoing) in the graph. */
    typedef graph_t::ArcIt arc_it_t;
    /**
     * Type of iterators over outgoing arcs in the graph. */
    typedef graph_t::OutArcIt out_arc_it_t;
    /**
     * Type of iterators over incomming arcs in the graph. */
    typedef graph_t::InArcIt in_arc_it_t;
    /**
     * Type of functions for computing the value of variables and
     * derivatives. The single function arguments are identified by their
     * position in the array given as function argument. */
    typedef std::function<double(std::vector<double>)> function_t;
    /**
     * Enum to distinguish different computation modes. */
    enum TYPE { VALUES,  ///< Compute values for variables and partial derivatives.
                FORWARD, ///< Compute differentials of nodes via forward accumulation.
                REVERSE  ///< Compute differentials of nodes via reverse accumulation.
    };

    //----members----//
protected:
    /**
     * Holds the graph-object if none was given in constructor. Otherwise it
     * holds an empty graph that is never manipulated. This is only relevant for
     * derived classes that use a non-const graph (like the ComputationalGraph
     * class). */
    graph_t dummy_graph;
private:

    /**
     * The graph-object. This is a reference to the graph object given in the
     * constructor. */
    const graph_t & graph;
    /**
     * Holds the label for every node. Labels are used to identify nodes when
     * computing values and derivatives. */
    graph_t::NodeMap<QString> node_labels;
    /**
     * Holds the value for every node. */
    graph_t::NodeMap<double> node_values;
    /**
     * Holds the differential for every node. */
    graph_t::NodeMap<double> node_differentials;
    /**
     * Holds the partial derivative of the source-node variabel w.r.t. the
     * target-node variable. */
    graph_t::ArcMap<double> arc_values;
    /**
     * Holds the list of variables for each node. This is a list of node labels
     * whose length must match the number of incoming arcs. The purpose is to
     * define a unique order of the relevant variables that can be used in
     * function definitions. This is necessary since the order in which
     * nodes/arcs are iterated over may change as the graph is modified. */
    graph_t::NodeMap<std::vector<QString>> node_variables;
    /**
     * The function to compute the value for each node. */
    graph_t::NodeMap<function_t> node_functions;
    /**
     * The function to compute the partial derivative for each arc. */
    graph_t::ArcMap<function_t> arc_functions;
    /**
     * The list of input nodes. That is, nodes that do not have incomming arcs
     * and thus correspond to independent variables. */
    std::vector<node_t> input_nodes;
    /**
     * The list of output nodes. That is, nodes that do not have outgoing arcs
     * and thus are the actual function value computed by the whole graph. */
    std::vector<node_t> output_nodes;

    //----methods----//
protected:
    /**
     * Default constructor. The default constructor is not public because it
     * constructs an empty graph that cannot be manipulated. It may be called by
     * derived classes though, which may then implement an interface for
     * maniuplating the graph (see e.g. the ComputationalGraph class).*/
    ComputationalConstGraph();
public:
    /**
     * Constructor with external graph-object. The graph being used the \e g,
     * dummy_graph is not used. */
    ComputationalConstGraph(const graph_t & graph);
    virtual ~ComputationalConstGraph() = default;
    /**
     * Compute all values. Assign values to input variables and compute values
     * and partial derivatives of all nodes and arcs. */
    virtual ComputationalConstGraph & compute_values(std::vector<double> values = std::vector<double>());
    /**
     * Compute values and total derivatives. Assign values and differentials to
     * input variables, compute values and partial derivatives, and compute
     * differentials and total derivatives of output variables via forward
     * accumulation. */
    virtual ComputationalConstGraph & forward_accumulation(std::vector<double> values = std::vector<double>(),
                                                           std::vector<double> differentials = std::vector<double>());
    /**
     * Compute values and total derivatives. Assign values to input variables
     * and differentials to output variables, compute values and partial
     * derivatives, and compute differentials and total derivatives of output
     * variables via reverse accumulation. */
    virtual ComputationalConstGraph & reverse_accumulation(std::vector<double> values = std::vector<double>(),
                                                           std::vector<double> differentials = std::vector<double>());
    /**
     * Like compute_values() but only update changed values. Actually
     * compute_values() is just a shorthand for calling update_values() on all
     * input nodes. */
    virtual ComputationalConstGraph & update_values(std::vector<node_t> nodes,
                                                    std::vector<double> values = std::vector<double>(),
                                                    bool input_nodes_only = true);
    /**
     * Like forward_accumulation() but only update changed values. Actually
     * forward_accumulation() is just a shorthand for calling compute_values()
     * and update_differentials_forward() on the input nodes. */
    virtual ComputationalConstGraph & update_differentials_forward(std::vector<node_t> nodes,
                                                                   std::vector<double> differentials = std::vector<double>(),
                                                                   bool input_nodes_only = true);
    /**
     * Like reverse_accumulation() but only update changed values. Actually
     * reverse_accumulation() is just a shorthand for calling compute_values()
     * on the input nodes and update_differentials_reverse() on the output
     * nodes. */
    virtual ComputationalConstGraph & update_differentials_reverse(std::vector<node_t> nodes,
                                                                   std::vector<double> differentials = std::vector<double>(),
                                                                   bool output_nodes_only = true);
    /**
     * Checks the derivatives via finite differences. The function changes every
     * node/variable by +/- \e delta. It then computes the nummerical derivative
     * for all dependent nodes using this a symetric finite difference and
     * compares the value to the function stored in the connecting arc. An error
     * is signaled (by printing a warning and returning false) if for any
     * derivative both the absolute deviation (i.e. the absulute value of the
     * difference between numerical and analytical derivative) is larger than \e
     * epsilon_absolute and the realtive deviation (i.e. the absolute value of
     * the quotient of numerical and analytical derivative) is larger than \e
     * 1+epsilon_relative. */
    virtual bool check_derivatives(std::vector<double> values = std::vector<double>(),
                                   double delta = 1e-5,
                                   double epsilon_absolute = 1e-10,
                                   double epsilon_relative = 1e-10);
    /**
     * Uses util::graph_to_pdf to plot the graph. */
    virtual ComputationalConstGraph & plot_graph(const char* file_name);
    /**
     * Set a node/variable.
     *
     * @param node_label The variable name used as ID for function calls of
     * dependend variables.
     *
     * @param node_variables List of node/variable labels used to construct the
     * array given to \e node_function as argument.
     *
     * @param node_function The function for computing the node/variable
     * value. */
    virtual ComputationalConstGraph & set_node(node_t node,
                                               QString node_label = "",
                                               std::vector<QString> node_variables = std::vector<QString>(),
                                               function_t node_function = [](std::vector<double>)->double{return NAN;});
    /**
     * Set an arc.
     *
     * @param node_function The function to compute the partial derivative of \e
     * to w.r.t. \e from. */
    virtual ComputationalConstGraph & set_arc(arc_t arc,
                                              function_t node_function = [](std::vector<double>)->double{return NAN;});
    /**
     * Returns the differentials of the input variables. */
    virtual std::vector<double> get_input_differentials() const;
    /**
     * Returns the differentials of the output variables. */
    virtual std::vector<double> get_output_differentials() const;
    /**
     * Returns the values of the output variables. */
    virtual std::vector<double> get_output_values() const;
    /**
     * Gets the input variables. */
    virtual std::vector<node_t> get_input_nodes() const { return input_nodes; }
    /**
     * Gets the output variables. */
    virtual std::vector<node_t> get_output_nodes() const { return output_nodes; }
    /**
     * Sets the input variables. */
    virtual ComputationalConstGraph & set_input_nodes(std::vector<node_t>);
    /**
     * Sets the output variables. */
    virtual ComputationalConstGraph & set_output_nodes(std::vector<node_t>);
    /**
     * Check the graph structure. This function checks three things: (1) whether
     * the graph is acyclic (2) whether the provided input/output nodes match
     * the sources/sinks of the graph (optionally the input and/or output nodes
     * may be reset but the order is not defined in that case) (3) whether the
     * list of variables provided for every node matches the actually present
     * nodes (the two lists must match exactly and must not contain any
     * duplicates). */
    virtual bool check_graph_structure(bool reset_input_nodes = false,
                                       bool reset_output_nodes = false);
    virtual QString get_node_label(node_t) const;
    virtual double get_node_value(node_t) const;
    virtual double get_node_differential(node_t) const;
    virtual double get_arc_value(arc_t) const;
    virtual void set_node_label(node_t,QString);
    virtual void set_node_value(node_t,double);
    virtual void set_node_function(node_t, std::vector<QString>, function_t);
    virtual void set_node_differential(node_t,double);
    virtual void set_arc_value(arc_t,double);
private:
    /**
     * Propagate values/differentials from changed nodes.
     *
     * @param p The mode of propagation.
     *
     * @param changed_nodes List of nodes whose value changed. */
    void propagate_values(TYPE p, std::vector<node_t> changed_nodes);
    /**
     * Assign values to variables. */
    void assign_values(std::vector<double> values, std::vector<node_t> nodes);
    /**
     * Assign values to differentials. */
    void assign_differentials(std::vector<double> values, std::vector<node_t> nodes);
    /**
     * Evaluate the given node.
     *
     * @param node The node to evaluate.
     *
     * @param e The mode of evaluation. */
    double evaluate_node(const node_t & node, TYPE e);
    /**
     * Check if \e nodes contain \e subnodes. Converts both vectors to sets and
     * uses std::includes(). */
    static bool includes(std::vector<node_t> nodes, std::vector<node_t> subnodes);
};

#endif /* COMPUTATIONALCONSTGRAPH_H_ */
