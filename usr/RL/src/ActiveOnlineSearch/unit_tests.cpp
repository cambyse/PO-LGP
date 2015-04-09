/** @file
 * \brief This file implements unit tests for the ActiveOnlineSearch
 * library and executable. */

#include <gtest/gtest.h>

#include <lemon/dfs.h>

#include "../util/pretty_printer.h"
#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../util/ND_vector.h"
#include "../util/graph_plotting.h"

#include "graph_util.h"

#define DEBUG_LEVEL 0
#include "../util/debug.h"

#include <ActiveOnlineSearch/ComputationalGraph.h>

using namespace ND_vector;
using util::Range;
using std::vector;
using std::cout;
using std::endl;

typedef lemon::ListDigraph graph_t;
typedef graph_t::Node Node;
typedef graph_t::Arc Arc;
typedef graph_t::NodeIt NodeIt;
typedef graph_t::ArcIt ArcIt;
typedef graph_t::OutArcIt OutArcIt;
typedef graph_t::InArcIt InArcIt;
template<class T>
using NodeMap = graph_t::NodeMap<T>;

TEST(ActiveOnlineSearch, ComputationalGraph) {

    /*=============================================
               Description of the Graph
      =============================================
     * We create a function R^3 --> R^2
     * */
    auto x = [](double alpha, double w, double t){
        return sin(w*exp(-alpha*sqrt(t))*sqrt(t));
    };
    auto y = [](double alpha, double w, double t){
        return cos(w*exp(-alpha*sqrt(t))*sqrt(t));
    };
     /* which we split up into sub-variables
     *
     *     v1 = sqrt(t)
     *     v2 = -alpha*v1
     *     v3 = exp(v2)
     *     v4 = w*v3*v1
     *
     * so that
     *
     *     x = sin(v4)
     *     y = cos(v4).
     *
     * The partial derivatives are
     *
     *     dv1/dt = 1/(2*sqrt(t))
     *
     *     dv2/dalpha = -v1
     *     dv2/dv1 = -alpha
     *
     *     dv3/dv2 = exp(v2)
     *
     *     dv4/dw = v1*v3
     *     dv4/dv1 = w*v3
     *     dv4/dv3 = w*v1
     *
     *     dx/dv4 = cos(v4)
     *
     *     dy/dv4 = -sin(v4).
     *
     * The total derivatives are:
     * */
    auto dx_dalpha = [](double alpha, double w, double t){
        return -cos(w*exp(-alpha*sqrt(t))*sqrt(t))*w*exp(-alpha*sqrt(t))*t;
    };
    auto dy_dalpha = [](double alpha, double w, double t){
        return sin(w*exp(-alpha*sqrt(t))*sqrt(t))*w*exp(-alpha*sqrt(t))*t;
    };

    auto dx_dw = [](double alpha, double w, double t){
        return cos(w*exp(-alpha*sqrt(t))*sqrt(t))*exp(-alpha*sqrt(t))*sqrt(t);
    };
    auto dy_dw = [](double alpha, double w, double t){
        return -sin(w*exp(-alpha*sqrt(t))*sqrt(t))*exp(-alpha*sqrt(t))*sqrt(t);
    };

    auto dx_dt = [](double alpha, double w, double t){
        double w_exp = w*exp(-alpha*sqrt(t));
        return (w_exp/(2*sqrt(t)))*cos(w_exp*sqrt(t))*(1-alpha*sqrt(t));
    };
    auto dy_dt = [](double alpha, double w, double t){
        double w_exp = w*exp(-alpha*sqrt(t));
        return -(w_exp/(2*sqrt(t)))*sin(w_exp*sqrt(t))*(1-alpha*sqrt(t));
    };

    /*=============================================
               Constructing the Graph
      =============================================*/


    typedef ComputationalGraph::node_t node_t;
    typedef lemon::ListDigraph graph_t;

    // use external graph object
    graph_t graph;
    ComputationalGraph cg(graph);

    // create the graph
    node_t alpha_node = cg.add_node("alpha");
    node_t w_node = cg.add_node("w");
    node_t t_node = cg.add_node("t");

    node_t v1_node = cg.add_node("v1", {"t"}, [](vector<double> v)->double{return sqrt(v[0]);});
    node_t v2_node = cg.add_node("v2", {"alpha","v1"}, [](vector<double> v)->double{return -v[0]*v[1];});
    node_t v3_node = cg.add_node("v3", {"v2"}, [](vector<double> v)->double{return exp(v[0]);});
    node_t v4_node = cg.add_node("v4", {"w","v3","v1"}, [](vector<double> v)->double{return v[0]*v[1]*v[2];});

    node_t x_node = cg.add_node("x", {"v4"}, [](vector<double> v)->double{return sin(v[0]);});
    node_t y_node = cg.add_node("y", {"v4"}, [](vector<double> v)->double{return cos(v[0]);});

    cg.add_arc(alpha_node, v2_node, [](vector<double> v)->double{return -v[1];});
    cg.add_arc(t_node, v1_node, [](vector<double> v)->double{return 1/(2*sqrt(v[0]));});
    cg.add_arc(w_node, v4_node, [](vector<double> v)->double{return v[1]*v[2];});
    cg.add_arc(v1_node, v2_node, [](vector<double> v)->double{return -v[0];});
    cg.add_arc(v1_node, v4_node, [](vector<double> v)->double{return v[0]*v[1];});
    cg.add_arc(v2_node, v3_node, [](vector<double> v)->double{return exp(v[0]);});
    cg.add_arc(v3_node, v4_node, [](vector<double> v)->double{return v[0]*v[2];});
    cg.add_arc(v4_node, x_node, [](vector<double> v)->double{return cos(v[0]);});
    cg.add_arc(v4_node, y_node, [](vector<double> v)->double{return -sin(v[0]);});

    /*=============================================
               Macros and Helper Tools
      =============================================*/

    // input values
#define VALUES 0.1,3,30

    // change values/differentials so that correct values are not computed
    // accidentally
#define DISTURB_VALUES cg.compute_values({0,0,0});
#define DISTURB_DIFFERENTIALS cg.forward_accumulation({VALUES},{1,1,1});
#define DISTURB_ALL cg.forward_accumulation({0,0,0},{1,1,1});

    // check if all entries are near upt to 10 significant digits (make a copy
    // to avoid calling functions multiple times if vec1 or vec2 are function
    // calls)
#define expect_near(vec1, vec2) {                               \
        auto v1 = vec1;                                         \
        auto v2 = vec2;                                         \
        EXPECT_EQ(v1.size(),v2.size());                         \
        for(int idx : Range(std::min(v1.size(),v2.size()))) {   \
            EXPECT_NEAR(v1[idx]/v2[idx], 1, 1e-10);             \
        }                                                       \
    }

    /*=============================================
                    General Tests
      =============================================*/

    // fill lists of input/output nodes
    vector<node_t> input_nodes({alpha_node, w_node, t_node});
    vector<node_t> output_nodes({x_node, y_node});
    cg.set_input_nodes(input_nodes);
    cg.set_output_nodes(output_nodes);

    // check graph structure and in-/output nodes
    {
        bool ok;
        EXPECT_TRUE(ok=cg.check_graph_structure()) <<
            "Graph structure is corrupt. Aborting tests.";
        if(!ok) return;
    }

    // check derivatives
    EXPECT_TRUE(cg.check_derivatives({VALUES}));


    /*=============================================
             Checking the Values in Detail
      =============================================*/

    // get the analytical values
    vec_double_1D output_values({ x(VALUES),         y(VALUES)});
    vec_double_1D output_diff_1({dx_dalpha(VALUES), dy_dalpha(VALUES)});
    vec_double_1D output_diff_2({dx_dw(VALUES),     dy_dw(VALUES)});
    vec_double_1D output_diff_3({dx_dt(VALUES),     dy_dt(VALUES)});
    vec_double_1D input_diff_1( {dx_dalpha(VALUES), dx_dw(VALUES), dx_dt(VALUES)});
    vec_double_1D input_diff_2( {dy_dalpha(VALUES), dy_dw(VALUES), dy_dt(VALUES)});

    // compute values and check
    DISTURB_ALL;
    expect_near(cg.compute_values({VALUES}).get_output_values(),
                output_values);
    DISTURB_ALL;
    expect_near(cg.update_values({VALUES},input_nodes).get_output_values(),
                output_values);


    // compute derivatives via forward accumulation
    DISTURB_DIFFERENTIALS;
    expect_near(cg.forward_accumulation({VALUES},{1,0,0}).get_output_differentials(),
                output_diff_1);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_forward({1,0,0},input_nodes).get_output_differentials(),
                output_diff_1);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.forward_accumulation({VALUES},{0,1,0}).get_output_differentials(),
                output_diff_2);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_forward({0,1,0},input_nodes).get_output_differentials(),
                output_diff_2);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.forward_accumulation({VALUES},{0,0,1}).get_output_differentials(),
                output_diff_3);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_forward({0,0,1},input_nodes).get_output_differentials(),
                output_diff_3);

    // compute derivatives via reverse accumulation
    DISTURB_DIFFERENTIALS;
    expect_near(cg.reverse_accumulation({VALUES},{1,0}).get_input_differentials(),
                input_diff_1);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_reverse({1,0},output_nodes).get_input_differentials(),
                input_diff_1);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.reverse_accumulation({VALUES},{0,1}).get_input_differentials(),
                input_diff_2);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_reverse({0,1},output_nodes).get_input_differentials(),
                input_diff_2);


    /*=============================================
                   Checking Updates
      =============================================*/

    // values
    {
        // change alpha
        auto v1 = cg.compute_values({0.1,3,30}).update_values({0.2},{alpha_node}).get_output_values();
        auto v2 = cg.compute_values({0.2,3,30}).get_output_values();
        EXPECT_EQ(v1,v2);
    }
    {
        // change w
        auto v1 = cg.compute_values({0.1,3,30}).update_values({4},{w_node}).get_output_values();
        auto v2 = cg.compute_values({0.1,4,30}).get_output_values();
        EXPECT_EQ(v1,v2);
    }
    {
        // change t
        auto v1 = cg.compute_values({0.1,3,30}).update_values({40},{t_node}).get_output_values();
        auto v2 = cg.compute_values({0.1,3,40}).get_output_values();
        EXPECT_EQ(v1,v2);
    }

    // forward accumulation
    {
        // change alpha
        auto v1 = cg.
            forward_accumulation({0.1,3,30},{0.1,0.1,0.1}).
            update_differentials_forward({0.2},{alpha_node}).
            get_output_differentials();
        auto v2 = cg.
            forward_accumulation({0.1,3,30},{0.2,0.1,0.1}).
            get_output_differentials();
        EXPECT_EQ(v1,v2);
    }
    {
        // change w
        auto v1 = cg.
            forward_accumulation({0.1,3,30},{0.1,0.1,0.1}).
            update_differentials_forward({0.2},{w_node}).
            get_output_differentials();
        auto v2 = cg.
            forward_accumulation({0.1,3,30},{0.1,0.2,0.1}).
            get_output_differentials();
        EXPECT_EQ(v1,v2);
    }
    {
        // change t
        auto v1 = cg.
            forward_accumulation({0.1,3,30},{0.1,0.1,0.1}).
            update_differentials_forward({0.2},{t_node}).
            get_output_differentials();
        auto v2 = cg.
            forward_accumulation({0.1,3,30},{0.1,0.1,0.2}).
            get_output_differentials();
        EXPECT_EQ(v1,v2);
    }

    // reverse accumulation
    {
        // change x
        auto v1 = cg.
            reverse_accumulation({0.1,3,30},{0.1,0.1}).
            update_differentials_reverse({0.2},{x_node}).
            get_input_differentials();
        auto v2 = cg.
            reverse_accumulation({0.1,3,30},{0.2,0.1}).
            get_input_differentials();
        EXPECT_EQ(v1,v2);
    }
    {
        // change y
        auto v1 = cg.
            reverse_accumulation({0.1,3,30},{0.1,0.1}).
            update_differentials_reverse({0.2},{y_node}).
            get_input_differentials();
        auto v2 = cg.
            reverse_accumulation({0.1,3,30},{0.1,0.2}).
            get_input_differentials();
        EXPECT_EQ(v1,v2);
    }
}


TEST(ActiveOnlineSearch, ComputationalGraph2) {

    // In the above graph, updates for reverse accumulation work if the
    // to_be_processed map is just set to false. Here they don't. So this test
    // complements the above to ensure that the to_be_processed map is correctly
    // computed.

    typedef ComputationalGraph::node_t node_t;
    typedef lemon::ListDigraph graph_t;

    // use default constructor here so the class manages its own graph object
    ComputationalGraph cg;

    // create the graph
    node_t in_node = cg.add_node("in");
    node_t a_node = cg.add_node("a", {"in"}, [](vector<double> v)->double{return v[0];});
    node_t b_node = cg.add_node("b", {"in"}, [](vector<double> v)->double{return v[0];});
    node_t c_node = cg.add_node("c", {"b"}, [](vector<double> v)->double{return v[0];});
    node_t out_node = cg.add_node("out", {"a","c"}, [](vector<double> v)->double{return v[0]+v[1];});

    cg.add_arc(in_node, a_node, [](vector<double> v)->double{return 1;});
    cg.add_arc(in_node, b_node, [](vector<double> v)->double{return 1;});
    cg.add_arc(b_node, c_node, [](vector<double> v)->double{return 1;});
    cg.add_arc(a_node, out_node, [](vector<double> v)->double{return 1;});
    cg.add_arc(c_node, out_node, [](vector<double> v)->double{return 1;});

    // check graph structure and in-/output nodes
    {
        bool ok;
        EXPECT_TRUE(ok=cg.check_graph_structure(true,true)) <<
            "Graph structure is corrupt. Aborting tests.";
        if(!ok) return;
    }

    // check derivatives
    EXPECT_TRUE(cg.check_derivatives({1}));

    // update reverse accumulation
    vec_double_1D v1 = cg.
        reverse_accumulation({1},{1}).
        update_differentials_reverse({2},{out_node}).
        get_input_differentials();
    vec_double_1D v2 = cg.
        reverse_accumulation({1},{2}).
        get_input_differentials();
    EXPECT_EQ(v1,v2);
}

graph_util::GraphPropagation<lemon::ListDigraph>
make_graph_for_graph_propagation_tests(lemon::ListDigraph & graph,
                                       lemon::ListDigraph::NodeMap<QString> & node_names) {
    // add nodes
#define NEW_NODE(name)                          \
    auto name = graph.addNode();                \
    node_names[name] = #name;                   \

    NEW_NODE(n0);
    NEW_NODE(n1);
    NEW_NODE(n2);
    NEW_NODE(n3);
    NEW_NODE(n4);
    NEW_NODE(n5);
    NEW_NODE(n6);
    NEW_NODE(n7);
    NEW_NODE(n8);
    NEW_NODE(n9);

#undef NEW_NODE

    // add arcs
    graph.addArc(n0,n3);

    graph.addArc(n1,n2);
    graph.addArc(n2,n3);
    graph.addArc(n3,n4);
    graph.addArc(n4,n5);
    graph.addArc(n5,n6);
    graph.addArc(n6,n7);
    graph.addArc(n7,n8);

    graph.addArc(n3,n9);
    graph.addArc(n9,n7);
    graph.addArc(n9,n4);

    graph.addArc(n8,n2);

    NodeMap<QString> node_property_map(graph);
    for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
        node_property_map[node] = QString("label=<<B>%1</B><BR/>id=%2>").
            arg(node_names[node]).
            arg(graph.id(node));
    }

    //graph_util::GraphPropagation<graph_t> graph_propagation(graph);
    auto graph_propagation = graph_util::GraphPropagationFactory(graph);
    for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
        if(node_names[node]=="n0" || node_names[node]=="n2") {
            graph_propagation.add_source(node);
            node_property_map[node] += " style=filled fillcolor=\"#ffaaaa\" penwidth=4";
        }
    }

    util::graph_to_pdf("graph.pdf", graph, "", &node_property_map);

    return graph_propagation;
}

TEST(GraphPropagation, Reachability) {
    graph_t graph;
    NodeMap<bool> reachable_map(graph);
    NodeMap<QString> node_names(graph);
    auto graph_propagation = make_graph_for_graph_propagation_tests(graph, node_names);
    graph_propagation.set_reachable_map(reachable_map).init();

    for(graph_t::NodeIt node(graph); node!=lemon::INVALID; ++node) {
        DEBUG_OUT(1,"Node " << node_names[node] << (reachable_map[node]?" was reached":" was not reached"));
        EXPECT_EQ(reachable_map[node], graph_propagation.is_reachable(node));
        if(node_names[node]=="n1") EXPECT_FALSE(reachable_map[node]);
        else EXPECT_TRUE(reachable_map[node]);
    }
}

TEST(GraphPropagation, Next) {
    graph_t graph;
    NodeMap<QString> node_names(graph);
    auto graph_propagation = make_graph_for_graph_propagation_tests(graph, node_names);
    graph_propagation.init();

    QString node_chain;
    for(auto next_node=graph_propagation.next();
        next_node!=lemon::INVALID;
        next_node=graph_propagation.next()) {
        node_chain+=node_names[next_node];
        DEBUG_OUT(1,"Next node: " << node_names[next_node]);
    }
    EXPECT_EQ("n3n9n4n5n6n7n8n2",node_chain) << "node_chain='" << node_chain << "'";
}

TEST(GraphPropagation, Diffusion) {

    // runs diffusion updates in a loopy graph with custom check-changed
    // function

    typedef lemon::ListDigraph graph_t;
    graph_t graph;
    NodeMap<QString> node_names(graph);
    auto graph_propagation = make_graph_for_graph_propagation_tests(graph, node_names);
    NodeMap<double> node_values(graph,0);
    NodeMap<double> old_node_values(graph,0);
    std::function<bool(Node node)> check_changed_function = [&](Node node)->bool{
        static NodeMap<bool> changed(graph,true);
        bool return_value = changed[node] || fabs(node_values[node]-old_node_values[node])>1e-10;
        changed[node] = false;
        old_node_values[node] = node_values[node];
        return return_value;
    };
    graph_propagation.set_check_change_function(check_changed_function).init();

    // set inital values and "sabotage" propagation by setting one node to the
    // value it has after the first iteration so propagation will stop if the
    // check-changed function is not chosen correctly
    for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
        if(node_names[node]=="n0" || node_names[node]=="n1") {
            node_values[node] = 1;
            old_node_values[node] = 1;
        }
        if(node_names[node]=="n3") {
            node_values[node] = 0.5;
            old_node_values[node] = 0.5;
        }
    }

    // propagate diffusion through graph
    int counter = 0;
    for(auto next_node=graph_propagation.next();
        next_node!=lemon::INVALID;
        next_node=graph_propagation.next()) {
        double val = 0;
        int pred = 0;
        for(InArcIt arc(graph,next_node); arc!=lemon::INVALID; ++arc) {
            val += node_values[graph.source(arc)];
            ++pred;
        }
        val /= pred;
        node_values[next_node] = val;
        DEBUG_OUT(1,"Next node: " << node_names[next_node]);
        DEBUG_OUT(1,"    value=" << node_values[next_node]);
        DEBUG_OUT(1,"    old value=" << old_node_values[next_node]);
        ++counter;
    }

    for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
        if(node_names[node]=="n0" || node_names[node]=="n1") {
            EXPECT_EQ(1,node_values[node]);
        } else {
            EXPECT_NEAR(1,node_values[node],1e-5) << "Failed for node " << node_names[node];
            EXPECT_NE(1,node_values[node]);
        }
    }
    EXPECT_GT(counter, 20);
}

TEST(GraphPropagation, ProcessingOrder) {
    graph_t graph;
    Node center = graph.addNode();
    Node first = graph.addNode();
    graph.addArc(center,first);
    Node prev, next = first;
    repeat(10) {
        prev = next;
        next = graph.addNode();
        graph.addArc(prev,next);
        graph.addArc(center,next);
    }
    util::graph_to_pdf("graph.pdf", graph);
    auto prop = graph_util::GraphPropagationFactory(graph);
    prop.add_source(center).init();
    QString node_chain;
    for(Node next=prop.next(); next!=lemon::INVALID; next=prop.next()) {
        DEBUG_OUT(1,"Next is " << graph.id(next));
        node_chain += QString("%1 ").arg(graph.id(next));
    }
    EXPECT_EQ(node_chain,"1 2 3 4 5 6 7 8 9 10 11 ") << "node_chain='" << node_chain << "'";
}

TEST(Lemon, Dfs) {

    typedef lemon::ListDigraph graph_t;
    typedef graph_t::Node Node;
    typedef graph_t::NodeIt NodeIt;

    graph_t graph;

    Node n0 = graph.addNode();
    Node n1 = graph.addNode();
    Node n2 = graph.addNode();
    Node n3 = graph.addNode();

    graph.addArc(n0, n3);
    graph.addArc(n1, n2);

    util::graph_to_pdf("graph.pdf", graph);

    std::vector<Node> list_1({n1,n0});
    std::vector<Node> list_2({n0,n1});
    for(auto list : {list_1,list_2}) {
        lemon::Dfs<graph_t> search(graph);
        search.init();
        for(Node node : list) {
            cout << "add node " << graph.id(node) << endl;
            search.addSource(node);
        }
        search.start();
        for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
            if(search.reached(node)) {
                cout << "node " << graph.id(node) << ": reached" << endl;
            } else {
                cout << "node " << graph.id(node) << ": NOT reached" << endl;
            }
        }
        cout << endl;
    }
}
