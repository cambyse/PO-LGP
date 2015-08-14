/** @file
 * \brief This file implements unit tests for the ActiveOnlineSearch
 * library and executable. */

#include <gtest/gtest.h>

#include <lemon/dfs.h>
#include <lemon/adaptors.h>

#include <sstream>

#include <map>
#include <deque>
#include <queue>

#include <util/pretty_printer.h>
#include <util/util.h>
#include <util/QtUtil.h>
#include <util/ND_vector.h>
#include <util/graph_plotting.h>
#include <util/return_tuple.h>

#include "graph_util.h"

#include <MCTS_Environment/AbstractEnvironment.h>
#include <MCTS_Environment/TemplateEnvironment.h>
#include <MCTS_Environment/IntegerEnvironment.h>

#include "Environment/GamblingHall.h"
#include "ComputationalGraph.h"
#include "TreeSearch/SearchTree.h"
#include "TreeSearch/NodeFinder.h"
#include "TreeSearch/TreePolicy.h"
#include "TreeSearch/ValueHeuristic.h"
#include "TreeSearch/BackupMethod.h"
#include "TreeSearch/PriorModels.h"
#include "TreeSearch/ActiveTreeSearch.h"

#define DEBUG_LEVEL 0
#include <util/debug.h>

using namespace ND_vector;
using util::Range;
using std::vector;
using std::tuple;
using std::pair;
using std::make_pair;
using std::cout;
using std::endl;
using lemon::INVALID;

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
    cg.compute_values({VALUES});
    EXPECT_TRUE(cg.check_derivatives());


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
    expect_near(cg.update_values(input_nodes,{VALUES}).get_output_values(),
                output_values);


    // compute derivatives via forward accumulation
    DISTURB_DIFFERENTIALS;
    expect_near(cg.forward_accumulation({VALUES},{1,0,0}).get_output_differentials(),
                output_diff_1);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_forward(input_nodes,{1,0,0}).get_output_differentials(),
                output_diff_1);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.forward_accumulation({VALUES},{0,1,0}).get_output_differentials(),
                output_diff_2);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_forward(input_nodes,{0,1,0}).get_output_differentials(),
                output_diff_2);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.forward_accumulation({VALUES},{0,0,1}).get_output_differentials(),
                output_diff_3);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_forward(input_nodes,{0,0,1}).get_output_differentials(),
                output_diff_3);

    // compute derivatives via reverse accumulation
    DISTURB_DIFFERENTIALS;
    expect_near(cg.reverse_accumulation({VALUES},{1,0}).get_input_differentials(),
                input_diff_1);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_reverse(output_nodes,{1,0}).get_input_differentials(),
                input_diff_1);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.reverse_accumulation({VALUES},{0,1}).get_input_differentials(),
                input_diff_2);
    DISTURB_DIFFERENTIALS;
    expect_near(cg.update_differentials_reverse(output_nodes,{0,1}).get_input_differentials(),
                input_diff_2);


    /*=============================================
                   Checking Updates
      =============================================*/

    // values
    {
        // change alpha
        auto v1 = cg.compute_values({0.1,3,30}).update_values({alpha_node},{0.2}).get_output_values();
        auto v2 = cg.compute_values({0.2,3,30}).get_output_values();
        EXPECT_EQ(v1,v2);
    }
    {
        // change w
        auto v1 = cg.compute_values({0.1,3,30}).update_values({w_node},{4}).get_output_values();
        auto v2 = cg.compute_values({0.1,4,30}).get_output_values();
        EXPECT_EQ(v1,v2);
    }
    {
        // change t
        auto v1 = cg.compute_values({0.1,3,30}).update_values({t_node},{40}).get_output_values();
        auto v2 = cg.compute_values({0.1,3,40}).get_output_values();
        EXPECT_EQ(v1,v2);
    }

    // forward accumulation
    {
        // change alpha
        auto v1 = cg.
            forward_accumulation({0.1,3,30},{0.1,0.1,0.1}).
            update_differentials_forward({alpha_node},{0.2}).
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
            update_differentials_forward({w_node},{0.2}).
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
            update_differentials_forward({t_node},{0.2}).
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
            update_differentials_reverse({x_node},{0.2}).
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
            update_differentials_reverse({y_node},{0.2}).
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
        update_differentials_reverse({out_node},{2}).
        get_input_differentials();
    vec_double_1D v2 = cg.
        reverse_accumulation({1},{2}).
        get_input_differentials();
    EXPECT_EQ(v1,v2);
}

TEST(ActiveOnlineSearch, ComputationalGraphDerivatives) {
    typedef ComputationalGraph::graph_t graph_t;
    typedef ComputationalGraph::node_t node_t;
    typedef ComputationalGraph::arc_t arc_t;

    // use default constructor here so the class manages its own graph object
    for(bool check : {false,true}) {
        ComputationalGraph cg;

        // create the graph
        node_t a_node = cg.add_node("a", {}, [](vector<double> v)->double{return 0;});
        node_t b_node = cg.add_node("b", {"a"}, [](vector<double> v)->double{
                DEBUG_OUT(1,"b:" << v[0]);
                return v[0];
            });
        node_t c_node = cg.add_node("c", {"a","b"}, [](vector<double> v)->double{
                DEBUG_OUT(1,"c:" << v[0] << "+" << v[1]);
                return v[0]+v[1];
            });
        arc_t a_b_arc = cg.add_arc(a_node, b_node, [](vector<double> v)->double{return 1;});
        arc_t a_c_arc = cg.add_arc(a_node, c_node, [](vector<double> v)->double{return 1;});
        arc_t b_c_arc = cg.add_arc(b_node, c_node, [](vector<double> v)->double{return 1;});

        cg.check_graph_structure(true,true);
        cg.compute_values({1});
        if(check) cg.check_derivatives();
        //cg.plot_graph("graph.pdf");
        EXPECT_EQ(cg.get_node_value(a_node),1);
        EXPECT_EQ(cg.get_node_value(b_node),1);
        EXPECT_EQ(cg.get_node_value(c_node),2);
        EXPECT_EQ(cg.get_arc_value(a_b_arc),1);
        EXPECT_EQ(cg.get_arc_value(a_c_arc),1);
        EXPECT_EQ(cg.get_arc_value(b_c_arc),1);
    }
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

    util::plot_graph("graph.pdf", graph, "", &node_property_map);

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

TEST(GraphPropagation, SmallLoops) {

    // construct graph with many small (and larger) loops and see if all nodes
    // get processed

    typedef lemon::ListDigraph graph_t;
    graph_t graph;
    auto old_n1 = graph.addNode();
    auto old_n2 = graph.addNode();
    graph.addArc(old_n1,old_n2);
    graph.addArc(old_n2,old_n1);
    for(int i=0; i<10; ++i) {
        auto n1 = graph.addNode();
        auto n2 = graph.addNode();
        graph.addArc(n1,old_n1);
        graph.addArc(old_n2,n2);
        old_n1 = n1;
        old_n2 = n2;
        graph.addArc(n1,n2);
        graph.addArc(n2,n1);
    }
    util::plot_graph("graph.pdf", graph, "", nullptr, "", nullptr, "", true, "neato");

    std::set<graph_t::Node> processed_nodes;
    auto graph_propagation = graph_util::GraphPropagationFactory(graph);
    graph_propagation.
        add_source(old_n1).
        add_source(old_n2).
        allow_incomplete_updates(true).
        init();
    for(auto next_node=graph_propagation.next();
        next_node!=lemon::INVALID;
        next_node=graph_propagation.next()) {
        DEBUG_OUT(1,"processing node " << graph.id(next_node));
        processed_nodes.insert(next_node);
    }

    for(NodeIt node(graph); node!=lemon::INVALID; ++node) {
        auto was_processed = processed_nodes.find(node)!=processed_nodes.end();
        EXPECT_TRUE(was_processed) <<
            "node " << graph.id(node) << " should be processed";
    }
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
    //util::graph_to_pdf("graph.pdf", graph);
    auto prop = graph_util::GraphPropagationFactory(graph);
    prop.add_source(center).init();
    QString node_chain;
    for(Node next=prop.next(); next!=lemon::INVALID; next=prop.next()) {
        DEBUG_OUT(1,"Next is " << graph.id(next));
        node_chain += QString("%1 ").arg(graph.id(next));
    }
    EXPECT_EQ(node_chain,"1 2 3 4 5 6 7 8 9 10 11 ") << "node_chain='" << node_chain << "'";
}

class TestEnvironment: public IntegerEnvironment {
public:
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto test_action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(test_action,nullptr);
        int action = test_action->action;
        if(action==0 || state==1) {
            return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), 0);
        } else {
            state = (state+1)%2;
            return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), 1);
        }
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new IntegerAction(0)),
                    action_handle_t(new IntegerAction(1))});
    }
    virtual bool has_terminal_state() const override {return false;}
    virtual bool is_terminal_state() const override {return false;}
    virtual bool is_deterministic() const override {return true;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "TestEnvironment";}
};

class MockSearchTree: public SearchTree {
public:
    MockSearchTree(std::shared_ptr<AbstractEnvironment> environment,
                   double discount,
                   std::shared_ptr<node_finder::NodeFinder> node_finder):
        SearchTree(environment,
                   discount,
                   node_finder) {}
    virtual ~MockSearchTree() = default;
    graph_t & get_graph() {return graph;}
    virtual void next_do() {
        using namespace return_tuple;
        typedef std::vector<action_handle_t> action_sequence_t;
        typedef std::deque<action_sequence_t> action_sequence_list_t;
        typedef std::tuple<action_handle_t,observation_handle_t> transition_t;
        typedef std::vector<transition_t> trajectory_t;
        typedef std::deque<trajectory_t> trajectory_list_t;
        // construct list with all possible trajectories of fixed length
        // starting at root node
        int length = 2;
        trajectory_list_t trajectory_list;
        {
            // construct a list with all possible action sequences of a given length
            auto action_list = environment->get_actions();
            action_sequence_list_t action_sequence_list(1,action_sequence_t());
            while((int)action_sequence_list.front().size()<length) {
                action_sequence_t short_action_sequence = action_sequence_list.front();
                DEBUG_OUT(1,"Expand action sequence of length " << short_action_sequence.size());
                for(action_handle_t action : action_list) {
                    DEBUG_OUT(1,"    add action " << *action);
                    action_sequence_t short_action_sequence_copy = short_action_sequence;
                    short_action_sequence_copy.push_back(action);
                    DEBUG_OUT(1,"        now has length " << short_action_sequence_copy.size());
                    DEBUG_OUT(1,"Now have " << action_sequence_list.size() << " action sequences");
                    action_sequence_list.push_back(short_action_sequence_copy);
                    DEBUG_OUT(1,"Now have " << action_sequence_list.size() << " action sequences");
                }
                DEBUG_OUT(1,"Now have " << action_sequence_list.size() << " action sequences");
                action_sequence_list.pop_front();
                DEBUG_OUT(1,"Now have " << action_sequence_list.size() << " action sequences");
            }
            // perform all action sequences
            DEBUG_OUT(1,"Have " << action_sequence_list.size() << " action sequences");
            for(action_sequence_t action_sequence : action_sequence_list) {
                DEBUG_OUT(1,"Next action sequence (length=" << action_sequence.size() << "):");
                environment->reset_state();
                trajectory_t trajectory;
                for(action_handle_t action : action_sequence) {
                    observation_handle_t observation;
                    reward_t reward;
                    t(observation,reward) = environment->transition(action);
                    transition_t transition(action,observation);
                    trajectory.push_back(transition);
                    DEBUG_OUT(1,"        <"
                              << *action << ","
                              << *observation << ">"
                        );
                }
                trajectory_list.push_back(trajectory);
            }
        }
        // roll out all trajectories from root node
        DEBUG_OUT(1,"Using trajectories:");
        for(trajectory_t trajectory : trajectory_list) {
            DEBUG_OUT(1,"    trajectory:");
            node_t current_observation_node = root_node;
            for(transition_t transition : trajectory) {
                action_handle_t action;
                observation_handle_t observation;
                t(action,observation) = transition;
                DEBUG_OUT(1,"        <"
                          << *action << ","
                          << *observation << ">"
                    );
                bool new_arc, new_node;
                // add action node
                node_t action_node;
                arc_t to_action_arc;
                t(to_action_arc, action_node, new_arc, new_node) = find_or_create_action_node(current_observation_node, action);
                // add observation node
                node_t observation_node;
                arc_t to_observation_arc;
                t(to_observation_arc, observation_node, new_arc, new_node) = find_or_create_observation_node(action_node, observation);
                // continue at new observation node
                current_observation_node = observation_node;
            }
        }
    }
    virtual action_handle_t recommend_action() const override {return action_handle_t();}
    virtual void write(std::ostream & out) const override {out<<"MockSearchTree()";}
};

TEST(SearchTree, NodeFinder_FullDAG_Overflow) {
    // The FullDAG class uses a depth map. To avoid recomputing all depth if the
    // root node is removed it uses a global offset that is incremented
    // instead. Therefore depth value just keep on increasing. To avoid an
    // overflow depth values are recomputed with zero offset once half-max of
    // the specific depth-type is reached. For unit tests (#ifdef UNIT_TESTS)
    // the class uses 'char' as depth type. This test check if this is actually
    // working.

    typedef node_finder::graph_t graph_t;
    typedef node_finder::node_t node_t;
    typedef node_finder::node_info_map_t node_info_map_t;
    typedef node_finder::action_handle_t action_handle_t;
    typedef node_finder::observation_handle_t observation_handle_t;
    typedef node_finder::FullDAG::depth_t depth_t;

    // set up everything
    auto environment = std::shared_ptr<TestEnvironment>(new TestEnvironment);
    graph_t graph;
    node_info_map_t node_info_map(graph);
    node_finder::FullDAG finder;
    finder.init(graph,node_info_map);

    // initialize
    TestEnvironment::reward_t reward;
    action_handle_t action = *(environment->get_actions().begin());
    observation_handle_t observation;
    return_tuple::t(observation,reward) = environment->transition(action);
    node_t action_node, observation_node_1, observation_node_2;
    observation_node_1 = graph.addNode();
    node_info_map[observation_node_1].observation = observation;
    finder.add_observation_node(observation_node_1);
    // to check offset removal
    depth_t old_depth = finder.get_true_depth(observation_node_1);
    bool found_discontinuity = false;
    for(int i=0; i<std::numeric_limits<depth_t>::max(); ++i) {
        // add new nodes to graph
        action_node = graph.addNode();
        observation_node_2 = graph.addNode();
        graph.addArc(observation_node_1,action_node);
        graph.addArc(action_node,observation_node_2);
        // add valid action and observation in node_info_map
        action = *(environment->get_actions().begin());
        return_tuple::t(observation,reward) = environment->transition(action);
        node_info_map[action_node].action = action;
        node_info_map[observation_node_2].observation = observation;
        // add new nodes in finder
        finder.add_action_node(action_node);
        finder.add_observation_node(observation_node_2);
        // check depth
        if(old_depth/2>finder.get_true_depth(observation_node_2)) {
            found_discontinuity = true;
        }
        old_depth = finder.get_true_depth(observation_node_2);
        // erase old nodes from finder
        finder.erase_observation_node(observation_node_1);
        finder.erase_action_node(action_node);
        // erase old nodes from graph
        graph.erase(observation_node_1);
        graph.erase(action_node);
        // swap observation nodes
        observation_node_1 = observation_node_2;
    }
    EXPECT_TRUE(found_discontinuity);
}

TEST(SearchTree, NodeFinder) {
    // This test checks the expected tree sizes for the different node finder
    // classes.
    using namespace return_tuple;
    typedef std::tuple<int,int,int,int,std::shared_ptr<node_finder::NodeFinder>> check_tuple;
    for(auto tup : {
            check_tuple(13,12,13,12,std::shared_ptr<node_finder::NodeFinder>(new node_finder::PlainTree)),
                check_tuple(12,12,12,12,std::shared_ptr<node_finder::NodeFinder>(new node_finder::ObservationTree)),
                check_tuple(11,12,11,12,std::shared_ptr<node_finder::NodeFinder>(new node_finder::FullDAG)),
                check_tuple(9,12,6,8,std::shared_ptr<node_finder::NodeFinder>(new node_finder::FullGraph))}) {

        std::shared_ptr<node_finder::NodeFinder> node_finder;
        int nodes_1, arcs_1, nodes_2, arcs_2;
        return_tuple::t(nodes_1, arcs_1, nodes_2, arcs_2, node_finder) = tup;

        // setup environment and search tree
        auto environment = std::shared_ptr<TestEnvironment>(new TestEnvironment);
        std::shared_ptr<MockSearchTree> mock_search_tree(new MockSearchTree(environment,
                                                                            0.9,
                                                                            node_finder));
        std::shared_ptr<AbstractSearchTree> search_tree(mock_search_tree);
        search_tree->init();

        // build tree
        search_tree->next();
        EXPECT_EQ(nodes_1,lemon::countNodes(mock_search_tree->get_graph()));
        EXPECT_EQ(arcs_1,lemon::countArcs(mock_search_tree->get_graph()));
        // search_tree->plot_graph("graph.pdf");
        // getchar();

        // perform transition
        environment->reset_state();
        AbstractEnvironment::action_handle_t action = environment->get_actions()[0];
        AbstractEnvironment::observation_handle_t observation;
        AbstractEnvironment::reward_t reward;
        t(observation,reward) = environment->transition(action);
        environment->make_current_state_default();

        // prune tree
        search_tree->update(action, observation);

        // build tree anew
        search_tree->next();
        EXPECT_EQ(nodes_2,lemon::countNodes(mock_search_tree->get_graph()));
        EXPECT_EQ(arcs_2,lemon::countArcs(mock_search_tree->get_graph()));
        // search_tree->plot_graph("graph.pdf");
        // getchar();
    }
}

class DepthEnvironment: public IntegerEnvironment {
public:
    DepthEnvironment(int reward_depth): reward_depth(reward_depth){}
    virtual ~DepthEnvironment() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto depth_action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(nullptr,depth_action);
        ++state;
        if(state>reward_depth) state = reward_depth;
        DEBUG_OUT(1,"Transition to " << state);
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)),
                                         state>=reward_depth?1:0);
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new IntegerAction(-1)),
                    action_handle_t(new IntegerAction(1))});
    }
    virtual bool has_terminal_state() const override {return false;}
    virtual bool is_terminal_state() const override {return false;}
    virtual bool is_deterministic() const override {return true;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "DepthEnvironment";}
private:
    int reward_depth;
};

TEST(MonteCarloTreeSearch, PlainTree) {
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    for(int depth : {5,6}) {
        auto environment = std::shared_ptr<AbstractEnvironment>(new DepthEnvironment(7));
        MonteCarloTreeSearch search(environment,
                                    1,
                                    std::shared_ptr<NodeFinder>(new PlainTree()),
                                    std::shared_ptr<TreePolicy>(new UCB1()),
                                    std::shared_ptr<ValueHeuristic>(new RolloutStatistics()),
                                    std::shared_ptr<BackupMethod>(new MonteCarlo()),
                                    MonteCarloTreeSearch::BACKUP_TYPE::TRACE);
        // this is important since it otherwise screws up the counts and the
        // tree will not be built uniform
        search.rollout_storage = MonteCarloTreeSearch::ROLLOUT_STORAGE::NONE;
        // do as many iterations as would be necessary to build a tree with
        // uniform depth
        int iterations = pow(2,depth+1)-2;
        for(int i=0; i<iterations; ++i) {
            search.next();
        }
        // compute some stuff to perform checks
        typedef MonteCarloTreeSearch::graph_t graph_t;
        typedef MonteCarloTreeSearch::node_t node_t;
        typedef MonteCarloTreeSearch::node_it_t node_it_t;
        typedef MonteCarloTreeSearch::arc_t arc_t;
        typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
        typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
        typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
        const graph_t & graph = search.get_graph();
        // find the root node
        node_t root;
        for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
            if(in_arc_it_t(graph,node)==lemon::INVALID) {
                root = node;
                break;
            }
        }
        // compute distances to all nodes
        graph_t::NodeMap<int> dists(graph);
        lemon::dfs(graph).distMap(dists).run(root);
        // find maximum distance, which is the depth of the tree
        int max_dist = 0;
        for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
            max_dist = std::max(max_dist,dists[node]);
        }
        EXPECT_EQ(2*iterations+1,lemon::countNodes(graph));
        EXPECT_EQ(2*iterations,lemon::countArcs(graph));
        if(depth==5) {
            EXPECT_EQ(2*depth,max_dist);
        } else if(depth==6) {
            // This relies on UCB1 focusing on the first reward and not
            // exploring uniformly. Increasing the exploration coefficient would
            // devaluate this test.
            EXPECT_LT(2*depth,max_dist);
        } else EXPECT_TRUE(false) << "Unhandled depth value";
        // visual output
        // search.plot_graph("graph.pdf");
        // getchar();
    }
}

class LineEnvironment: public IntegerEnvironment {
public:
    LineEnvironment(int line_width): line_width(line_width) {}
    virtual ~LineEnvironment() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto line_action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(nullptr,line_action);
        state += line_action->action;
        if(state>line_width) state = line_width;
        if(-state>line_width) state = -line_width;
        DEBUG_OUT(1,"Transition to " << state);
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), state);
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new IntegerAction(-1)),
                    action_handle_t(new IntegerAction(0)),
                    action_handle_t(new IntegerAction(1))});
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state==line_width;}
    virtual bool is_deterministic() const override {return true;}
    virtual bool has_max_reward() const override {return false;}
    virtual reward_t max_reward() const override {return 0;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "LineEnvironment";}
private:
    int line_width;
};

TEST(MonteCarloTreeSearch, NodeFinder) {
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    vector<int> actual_depths;
    vector<int> node_counts;
    vector<int> arc_counts;
    int depth = 3;
    for(auto node_finder : {std::shared_ptr<NodeFinder>(new PlainTree()),
                std::shared_ptr<NodeFinder>(new ObservationTree()),
                std::shared_ptr<NodeFinder>(new FullDAG()),
                std::shared_ptr<NodeFinder>(new FullGraph())}) {
        auto environment = std::shared_ptr<AbstractEnvironment>(new LineEnvironment(2));
        MonteCarloTreeSearch search(environment,
                                    1,
                                    node_finder,
                                    std::shared_ptr<TreePolicy>(new UCB1(1e10)),
                                    std::shared_ptr<ValueHeuristic>(new RolloutStatistics()),
                                    std::shared_ptr<BackupMethod>(new MonteCarlo()),
                                    MonteCarloTreeSearch::BACKUP_TYPE::TRACE);
        search.rollout_storage = MonteCarloTreeSearch::ROLLOUT_STORAGE::NONE;
        // do as many iterations as would be necessary to build a tree with
        // uniform depth
        int iterations = (pow(3,depth+1)-1)/2 - 1;
        for(int i=0; i<iterations; ++i) {
            search.next();
            // search.plot_graph("graph.pdf");
            // getchar();
        }
        // compute some stuff to perform checks
        typedef MonteCarloTreeSearch::graph_t graph_t;
        typedef MonteCarloTreeSearch::node_t node_t;
        typedef MonteCarloTreeSearch::node_it_t node_it_t;
        typedef MonteCarloTreeSearch::arc_t arc_t;
        typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
        typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
        typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
        const graph_t & graph = search.get_graph();
        // find the root node
        node_t root = lemon::INVALID;
        for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
            if(in_arc_it_t(graph,node)==lemon::INVALID) {
                root = node;
                break;
            }
        }
        // compute distances to all nodes
        graph_t::NodeMap<int> dists(graph);
        lemon::dfs(graph).distMap(dists).run(root);
        // find maximum distance, which is the depth of the tree
        int max_dist = 0;
        for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
            max_dist = std::max(max_dist,dists[node]);
        }
        actual_depths.push_back(max_dist);
        node_counts.push_back(lemon::countNodes(graph));
        arc_counts.push_back(lemon::countArcs(graph));
        // visual output
        // search.plot_graph("graph.pdf");
        // getchar();
    }
    // some checks
    for(int idx=0; idx<4; ++idx) {
        vector<int> actual_depths_expected({6,8,-1,-1});
        vector<int> node_counts_expected({73,74,-1,21});
        vector<int> arc_counts_expected({72,74,-1,30});
        if(idx==0 || idx==1) { // PlainTree, ObservationTree
            EXPECT_EQ(actual_depths_expected[idx],actual_depths[idx]);
            EXPECT_EQ(node_counts_expected[idx],node_counts[idx]);
            EXPECT_EQ(arc_counts_expected[idx],arc_counts[idx]);
        } else if(idx==2) { // FullDAG
            // is random size but in any case the largest because it adds more
            // action nodes (without terminating a rollout because it reuses the
            // observation nodes)
            EXPECT_LT(actual_depths[idx-1],actual_depths[idx]);
            EXPECT_LT(actual_depths[idx-2],actual_depths[idx]);
            EXPECT_LT(actual_depths[idx+1],actual_depths[idx]);
            EXPECT_LT(node_counts[idx-1],node_counts[idx]);
            EXPECT_LT(node_counts[idx-2],node_counts[idx]);
            EXPECT_LT(node_counts[idx+1],node_counts[idx]);
            EXPECT_LT(arc_counts[idx-1],arc_counts[idx]);
            EXPECT_LT(arc_counts[idx-2],arc_counts[idx]);
            EXPECT_LT(arc_counts[idx+1],arc_counts[idx]);
        } else if(idx==3) { // FullGraph
            EXPECT_TRUE(actual_depths[idx]==7
                        || actual_depths[idx]==8
                        || actual_depths[idx]==9) << "Depth is " << actual_depths[idx];
            EXPECT_EQ(node_counts_expected[idx],node_counts[idx]);
            EXPECT_EQ(arc_counts_expected[idx],arc_counts[idx]);
        } else EXPECT_TRUE(false) << "This line should never occur";
    }
}

class FiniteLineEnvironment: public IntegerEnvironment {
public:
    FiniteLineEnvironment(int line_width): line_width(line_width) {}
    virtual ~FiniteLineEnvironment() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto line_action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(nullptr,line_action);
        //state += (line_action->action>0?1:0);
        state += line_action->action;
        if(state>line_width) state = line_width;
        if(state<0) state = 0;
        reward_t reward = state==line_width?1:0;
        DEBUG_OUT(1,"Transition to " << state);
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), reward);
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new IntegerAction(-1)),
                    action_handle_t(new IntegerAction(0)),
                    action_handle_t(new IntegerAction(1))});
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state==line_width;}
    virtual bool is_deterministic() const override {return true;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "FiniteLineEnvironment";}
private:
    int line_width;
};

TEST(MonteCarloTreeSearch, BackupType) {
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    for(auto finder_iterations : vector<pair<std::shared_ptr<NodeFinder>, int>>{{
                {std::shared_ptr<NodeFinder>(new PlainTree()),12},
                {std::shared_ptr<NodeFinder>(new ObservationTree()),11},
                {std::shared_ptr<NodeFinder>(new FullDAG()),100},
                {std::shared_ptr<NodeFinder>(new FullGraph()),100}
            }}) {
        for(auto backup_type : {
                MonteCarloTreeSearch::BACKUP_TYPE::TRACE,
                    MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE
                    }) {
            RETURN_TUPLE(std::shared_ptr<NodeFinder>, node_finder, int, iterations) = finder_iterations;
            auto environment = std::shared_ptr<AbstractEnvironment>(new FiniteLineEnvironment(2));
            MonteCarloTreeSearch search(environment,
                                        0.5,
                                        node_finder,
                                        std::shared_ptr<TreePolicy>(new UCB1(1e10)),
                                        std::shared_ptr<ValueHeuristic>(new RolloutStatistics(0)),
                                        std::shared_ptr<BackupMethod>(new Bellman(nullptr,0)),
                                        backup_type);
            IF_DEBUG(1) {
                cout << "Testing" << endl;
                cout << "    " << typeid(*node_finder).name() << endl;
                cout << "    " << typeid(backup_type).name() << ": " << (int)backup_type << endl;
            } else {
                cout << "." << std::flush;
            }
            // for the tests
            typedef MonteCarloTreeSearch::graph_t graph_t;
            typedef MonteCarloTreeSearch::node_t node_t;
            typedef MonteCarloTreeSearch::node_it_t node_it_t;
            typedef MonteCarloTreeSearch::arc_t arc_t;
            typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
            typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
            typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
            const graph_t & graph = search.get_graph();
            auto & info = search.get_node_info_map();
            auto & mcts_info = search.get_mcts_node_info_map();
            int non_matching_action_values = 0;

            // iterate and sum up non-matching action value in each step
            for(int i=0; i<iterations; ++i) {
                search.next();
                // find non-matching action values
                DEBUG_OUT(2,"Iteration " << i);
                for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
                    int local_non_matching_action_values = 0;
                    if(info[node].type==MonteCarloTreeSearch::ACTION_NODE) continue;
                    double value = -1;
                    for(in_arc_it_t arc(graph,node); arc!=lemon::INVALID; ++arc) {
                        if(value==-1) {
                            value = mcts_info[graph.source(arc)].value;
                        } else {
                            if(value!=mcts_info[graph.source(arc)].value) {
                                ++local_non_matching_action_values;
                            }
                        }
                    }
                    IF_DEBUG(2) {
                        if(local_non_matching_action_values>0) {
                            DEBUG_OUT(2,"        " << local_non_matching_action_values <<
                                      " non-matching action values for node " << graph.id(node));
                            // search.plot_graph("graph.pdf");
                            // getchar();
                        }
                    }
                    non_matching_action_values += local_non_matching_action_values;
                }
                DEBUG_OUT(2,"    sum of non-matching action-values " << non_matching_action_values);
                // search.plot_graph("graph.pdf");
                // getchar();
            }

            // check number of non-matching action values
            if(backup_type==MonteCarloTreeSearch::BACKUP_TYPE::TRACE) {
                if(typeid(*node_finder)==typeid(PlainTree)) {
                    EXPECT_EQ(non_matching_action_values,0);
                } else {
                    EXPECT_GT(non_matching_action_values,0);
                }
            } else if(backup_type==MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE) {
                EXPECT_EQ(non_matching_action_values,0);
                 } else EXPECT_TRUE(false) << "This line should not be reached";
            // visual output
            // search.plot_graph("graph.pdf");
            // getchar();
        }
    }
    IF_DEBUG(1) {/* do nothing*/}
    else cout << endl;
}

class SimpleTestEnvironment: public IntegerEnvironment {
public:
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto x = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(x,nullptr);
        reward_t reward;
        switch(state) {
        case 0:
            reward = 0;
            if(drand48()<0.1*x->action) {
                state = 2;
            } else {
                state = 1;
            }
            break;
        case 1:
            reward = 0;
            state = 3;
            break;
        case 2:
            reward = 1;
            state = 3;
            break;
        case 3:
            reward = 0;
            state = 4;
            break;
        default:
            EXPECT_TRUE(false) << "This line should never be reached";
        }
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), reward);
    }
    virtual action_container_t get_actions() override {
        if(state==0) {
            return action_container_t({
                        action_handle_t(new IntegerAction(0)),
                            //action_handle_t(new IntegerAction(1)),
                        action_handle_t(new IntegerAction(2)),
                            //action_handle_t(new IntegerAction(3)),
                        action_handle_t(new IntegerAction(4)),
                            //action_handle_t(new IntegerAction(5)),
                        action_handle_t(new IntegerAction(6)),
                            //action_handle_t(new IntegerAction(7)),
                        action_handle_t(new IntegerAction(8)),
                            //action_handle_t(new IntegerAction(9)),
                        action_handle_t(new IntegerAction(10)
                            )});
        } else {
            return action_container_t({action_handle_t(new IntegerAction(0))});
        }
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state==4;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "SimpleTestEnvironment";}
};

TEST(MonteCarloTreeSearch, Backup2) {
    // initializde environment and search tree
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    using namespace prior_models;
    for(auto node_finder : {
            std::shared_ptr<NodeFinder>(new PlainTree()),
                std::shared_ptr<NodeFinder>(new ObservationTree()),
                std::shared_ptr<NodeFinder>(new FullDAG()),
                std::shared_ptr<NodeFinder>(new FullGraph())
                }) {
        for(auto tree_policy: {
                std::shared_ptr<TreePolicy>(new Uniform()),
                    std::shared_ptr<TreePolicy>(new UCB1(1e10)),
                    std::shared_ptr<TreePolicy>(new UCB_Variance(1e10))
                    }) {
            for(auto value_heuristic : {std::shared_ptr<ValueHeuristic>(new RolloutStatistics(1))}) {
                for(auto backup_method : {
                        std::shared_ptr<BackupMethod>(new Bellman(nullptr,1)),
                            std::shared_ptr<BackupMethod>(new MonteCarlo(1))
                            }) {
                    for(auto backup_type : {
                            MonteCarloTreeSearch::BACKUP_TYPE::TRACE,
                                MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE
                                }) {
                        auto environment = std::shared_ptr<AbstractEnvironment>(new SimpleTestEnvironment());
                        double discount = 0.5;
                        MonteCarloTreeSearch search(environment,
                                                    discount,
                                                    node_finder,
                                                    tree_policy,
                                                    value_heuristic,
                                                    backup_method,
                                                    backup_type);
                        search.rollout_storage = MonteCarloTreeSearch::ROLLOUT_STORAGE::CONDENSED;
                        IF_DEBUG(1) {
                            cout << "Testing" << endl;
                            cout << "    " << typeid(*node_finder).name() << endl;
                            cout << "    " << typeid(*tree_policy).name() << endl;
                            cout << "    " << typeid(*value_heuristic).name() << endl;
                            cout << "    " << typeid(*backup_method).name() << endl;
                            cout << "    " << typeid(backup_type).name() << ": " << (int)backup_type << endl;
                        } else {
                            cout << "." << std::flush;
                        }
                        // do rollouts
                        int rollout_n = 1000;
                        repeat(rollout_n) {
                            search.next();
                            // search.plot_graph("graph.pdf");
                            // getchar();
                        }
                        // check action values
                        for(auto action_value : search.get_action_values()) {
                            RETURN_TUPLE(MonteCarloTreeSearch::action_handle_t, action,
                                         double, empirical_value) = action_value;
                            auto simple_action = std::dynamic_pointer_cast<const SimpleTestEnvironment::IntegerAction>(action);
                            EXPECT_NE(simple_action, nullptr);
                            double analytical_value = 0.1*simple_action->action*discount;
                            EXPECT_NEAR(analytical_value, empirical_value, 0.1) <<
                                "This is a statistical test with a small chance of false alarms -- recheck";
                        }
                        // check rollout counts
                        typedef MonteCarloTreeSearch::graph_t graph_t;
                        typedef MonteCarloTreeSearch::node_t node_t;
                        typedef MonteCarloTreeSearch::node_it_t node_it_t;
                        typedef MonteCarloTreeSearch::arc_t arc_t;
                        typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
                        typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
                        typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
                        const graph_t & graph = search.get_graph();
                        auto & node_info_map = search.get_node_info_map();
                        auto & mcts_node_info_map = search.get_mcts_node_info_map();
                        int rollout_counts_1 = 0;
                        int rollout_counts_2 = 0;
                        for(node_it_t node(graph); node!=INVALID; ++node) {
                            // ignore root node
                            if(in_arc_it_t(graph,node)==INVALID) continue;
                            // ignore action nodes
                            if(node_info_map[node].type==SearchTree::ACTION_NODE) continue;
                            // all observation nodes at the same depth
                            // (i.e. with observation 3 or 1/2) should in sum
                            // contain all rollouts ever made (checked below,
                            // here we compute the sum)
                            auto observation = node_info_map[node].observation;
                            auto simple_observation = std::dynamic_pointer_cast<const SimpleTestEnvironment::IntegerObservation>(observation);
                            EXPECT_NE(simple_observation,nullptr);
                            if(simple_observation->observation==1 || simple_observation->observation==2) {
                                rollout_counts_1 += mcts_node_info_map[node].rollout_counts;
                            } else if(simple_observation->observation==3) {
                                rollout_counts_2 += mcts_node_info_map[node].rollout_counts;
                            }
                        }
                        EXPECT_EQ(rollout_counts_1,rollout_n);
                        EXPECT_EQ(rollout_counts_2,rollout_n);
                        // plot graph before pruning
                        // search.plot_graph("graph.pdf");
                        // do first step
                        environment->reset_state();
                        {
                            auto action = search.recommend_action();
                            auto simple_action = std::dynamic_pointer_cast<const SimpleTestEnvironment::IntegerAction>(action);
                            EXPECT_NE(simple_action, nullptr);
                            EXPECT_EQ(simple_action->action,10);
                            RETURN_TUPLE(observation_handle_t, observation,
                                         reward_t, reward) = environment->transition(action);
                            auto simple_observation = std::dynamic_pointer_cast<const SimpleTestEnvironment::IntegerObservation>(observation);
                            EXPECT_NE(simple_observation, nullptr);
                            EXPECT_EQ(simple_observation->observation,2);
                            EXPECT_EQ(reward,0);
                            search.update(action,observation);
                        }
                        // do second step
                        {
                            auto action = search.recommend_action();
                            auto simple_action = std::dynamic_pointer_cast<const SimpleTestEnvironment::IntegerAction>(action);
                            EXPECT_NE(simple_action, nullptr);
                            EXPECT_EQ(simple_action->action,0);
                            RETURN_TUPLE(observation_handle_t, observation,
                                         reward_t, reward) = environment->transition(action);
                            auto simple_observation = std::dynamic_pointer_cast<const SimpleTestEnvironment::IntegerObservation>(observation);
                            EXPECT_NE(simple_observation, nullptr);
                            EXPECT_EQ(simple_observation->observation,3);
                            EXPECT_EQ(reward,1);
                            search.update(action,observation);
                        }
                        // getchar();
                    }
                }
            }
        }
    }
    IF_DEBUG(1) {/* do nothing*/}
    else cout << endl;
}

class SplitEnvironment: public IntegerEnvironment {
public:
    SplitEnvironment() {state = -5; default_state = -5;}
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto x = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(x,nullptr);
        reward_t reward = 0;
        if(state==0 && x->action==1) {
            if(rand()%3==0) {
                state = 10;
            } else {
                state = 1;
            }
        } else if(state==0 && x->action==-1) {
            if(rand()%3==0) {
                state = 1;
            } else {
                state = 10;
            }
        } else if(state==5 || state==50) {
            state = 100;
        } else if(state<10) {
            reward = -0.1;
            state += 1;
        } else if(state<100) {
            reward = +0.1;
            state += 10;
        } else {
            state += 1;
        }
        if(is_terminal_state()) {
            reward = 1;
        }
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), reward);
    }
    virtual action_container_t get_actions() override {
        if(state==0) {
            return action_container_t({
                    action_handle_t(new IntegerAction(+1)),
                        action_handle_t(new IntegerAction(-1)
                            )});
        } else {
            return action_container_t({action_handle_t(new IntegerAction(0))});
        }
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state==105;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "SplitEnvironment";}
};

TEST(MonteCarloTreeSearch, RolloutTransfer) {
    // initialize environment and search tree
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    using namespace prior_models;
    for(auto node_finder : {
            std::shared_ptr<NodeFinder>(new PlainTree()),
                std::shared_ptr<NodeFinder>(new ObservationTree()),
                std::shared_ptr<NodeFinder>(new FullDAG()),
                std::shared_ptr<NodeFinder>(new FullGraph())
                }) {
        for(auto tree_policy: {
                std::shared_ptr<TreePolicy>(new Uniform()),
                    std::shared_ptr<TreePolicy>(new UCB1(1e10)),
                    std::shared_ptr<TreePolicy>(new UCB_Variance(1e10))
                    }) {
            for(auto value_heuristic : {std::shared_ptr<ValueHeuristic>(new RolloutStatistics(.1))}) {
                for(auto backup_method : {
                        std::shared_ptr<BackupMethod>(new Bellman(nullptr,.1)),
                            std::shared_ptr<BackupMethod>(new MonteCarlo(.1))
                            }) {
                    for(auto backup_type : {
                            MonteCarloTreeSearch::BACKUP_TYPE::TRACE,
                                MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE
                                }) {
                        auto environment = std::shared_ptr<AbstractEnvironment>(new SplitEnvironment());
                        double discount = 0.9;
                        MonteCarloTreeSearch search(environment,
                                                    discount,
                                                    node_finder,
                                                    tree_policy,
                                                    value_heuristic,
                                                    backup_method,
                                                    backup_type);
                        search.rollout_storage = MonteCarloTreeSearch::ROLLOUT_STORAGE::CONDENSED;
                        IF_DEBUG(1) {
                            cout << "Testing" << endl;
                            cout << "    " << typeid(*node_finder).name() << endl;
                            cout << "    " << typeid(*tree_policy).name() << endl;
                            cout << "    " << typeid(*value_heuristic).name() << endl;
                            cout << "    " << typeid(*backup_method).name() << endl;
                            cout << "    " << typeid(backup_type).name() << ": " << (int)backup_type << endl;
                        } else {
                            cout << "." << std::flush;
                        }
                        // do rollouts
                        int rollout_n = 500;
                        repeat(rollout_n) {
                            search.next();
                            // search.plot_graph("graph.pdf");
                            // getchar();
                        }
                        // check rollout counts
                        typedef MonteCarloTreeSearch::graph_t graph_t;
                        typedef MonteCarloTreeSearch::node_t node_t;
                        typedef MonteCarloTreeSearch::node_it_t node_it_t;
                        typedef MonteCarloTreeSearch::arc_t arc_t;
                        typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
                        typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
                        typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
                        const graph_t & graph = search.get_graph();
                        auto & node_info_map = search.get_node_info_map();
                        auto & mcts_node_info_map = search.get_mcts_node_info_map();
                        std::map<int,int> rollout_counts;
                        for(node_it_t node(graph); node!=INVALID; ++node) {
                            // ignore root node
                            if(in_arc_it_t(graph,node)==INVALID) continue;
                            // ignore action nodes (check together with observation node)
                            if(node_info_map[node].type==SearchTree::ACTION_NODE) continue;
                            // check observation value
                            auto observation = node_info_map[node].observation;
                            auto split_observation = std::dynamic_pointer_cast<const SplitEnvironment::IntegerObservation>(observation);
                            EXPECT_NE(split_observation,nullptr);
                            // convert observations from "small" branch to
                            // those of "large" branch
                            int obs = split_observation->observation;
                            if(obs<10) obs *= 10;
                            rollout_counts[obs] += mcts_node_info_map[node].rollout_counts;
                        }
                        for(auto counts : rollout_counts) {
                            EXPECT_EQ(counts.second,rollout_n);
                        }
                        // search.plot_graph("graph.pdf");
                        // getchar();
                    }
                }
            }
        }
    }
    IF_DEBUG(1) {/* do nothing*/}
    else cout << endl;
}

class LoopyEnvironment: public IntegerEnvironment {
public:
    LoopyEnvironment(int loop_size): loop_size(loop_size) {}
    virtual ~LoopyEnvironment() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto loopy_action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(nullptr,loopy_action);
        if(abs(state)==loop_size) {
            state = -(loop_size-1);
        } else {
            if(loopy_action->action==-1) {
                state = -state;
            } else if(loopy_action->action==1) {
                state += 1;
            } else EXPECT_TRUE(false) << "This line should never be reached.";
        }
        reward_t reward = state==loop_size?1:-0.1;
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), reward);
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new IntegerAction(-1)),
                    action_handle_t(new IntegerAction(1))});
    }
    virtual bool has_terminal_state() const override {return false;}
    virtual bool is_terminal_state() const override {return false;}
    virtual bool is_deterministic() const override {return true;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "LoopyEnvironment";}
private:
    int loop_size;
};

TEST(MonteCarloTreeSearch, LoopyPropagation) {
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    for(auto finder_iterations : vector<pair<std::shared_ptr<NodeFinder>, int>>{{
                {std::shared_ptr<NodeFinder>(new PlainTree()),1000},
                {std::shared_ptr<NodeFinder>(new ObservationTree()),1000},
                {std::shared_ptr<NodeFinder>(new FullDAG()),100},
                {std::shared_ptr<NodeFinder>(new FullGraph()),1000}
            }}) {
        for(auto backup_type : {
                MonteCarloTreeSearch::BACKUP_TYPE::TRACE,
                    MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE
                    }) {
            RETURN_TUPLE(std::shared_ptr<NodeFinder>, node_finder, int, iterations) = finder_iterations;
            auto environment = std::shared_ptr<AbstractEnvironment>(new LoopyEnvironment(4));
            MonteCarloTreeSearch search(environment,
                                        0.5,
                                        node_finder,
                                        std::shared_ptr<TreePolicy>(new UCB1(2)),
                                        std::shared_ptr<ValueHeuristic>(new RolloutStatistics(0)),
                                        std::shared_ptr<BackupMethod>(new Bellman(nullptr,0)),
                                        backup_type);
            IF_DEBUG(1) {
                cout << "Testing" << endl;
                cout << "    " << typeid(*node_finder).name() << endl;
                cout << "    " << typeid(backup_type).name() << ": " << (int)backup_type << endl;
            } else {
                cout << "." << std::flush;
            }
            // for the tests
            typedef MonteCarloTreeSearch::graph_t graph_t;
            typedef MonteCarloTreeSearch::node_t node_t;
            typedef MonteCarloTreeSearch::node_it_t node_it_t;
            typedef MonteCarloTreeSearch::arc_t arc_t;
            typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
            typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
            typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
            const graph_t & graph = search.get_graph();
            auto & info = search.get_node_info_map();
            auto & mcts_info = search.get_mcts_node_info_map();

            // iterate and sum up non-matching action value in each step
            int non_matching_action_values = 0;
            for(int i=0; i<iterations; ++i) {
                search.next();
                // find non-matching action values
                DEBUG_OUT(2,"Iteration " << i);
                for(node_it_t node(graph); node!=lemon::INVALID; ++node) {
                    DEBUG_OUT(3,"    checking node " << graph.id(node));
                    int local_non_matching_action_values = 0;
                    if(info[node].type==MonteCarloTreeSearch::ACTION_NODE) continue;
                    double value = -1;
                    for(in_arc_it_t arc(graph,node); arc!=lemon::INVALID; ++arc) {
                        if(value==-1) {
                            value = mcts_info[graph.source(arc)].value;
                            DEBUG_OUT(3,"        use value " << value << " from node " <<
                                      graph.id(graph.source(arc)) << " as reference");
                        } else {
                            if(fabs(value-mcts_info[graph.source(arc)].value)>1e-5) {
                                DEBUG_OUT(3,"        value " << value << " from node " <<
                                          graph.id(graph.source(arc)) << " does not match");
                                ++local_non_matching_action_values;
                            } else {
                                DEBUG_OUT(3,"        value " << mcts_info[graph.source(arc)].value <<
                                          " from node " << graph.id(graph.source(arc)) << " matches");
                            }
                        }
                    }
                    IF_DEBUG(2) {
                        if(local_non_matching_action_values>0) {
                            DEBUG_OUT(2,"        " << local_non_matching_action_values <<
                                      " non-matching action values for node " << graph.id(node));
                        }
                    }
                    non_matching_action_values += local_non_matching_action_values;
                }
                DEBUG_OUT(2,"    sum of non-matching action-values " << non_matching_action_values);
                // visual output
                // if(typeid(*node_finder).name()==typeid(FullGraph).name()) {
                //     search.plot_graph("graph.pdf","neato");
                // } else {
                //     search.plot_graph("graph.pdf","dot");
                // }
                // getchar();
            }

            // check number of non-matching action values
            if(backup_type==MonteCarloTreeSearch::BACKUP_TYPE::TRACE) {
                if(typeid(*node_finder)==typeid(PlainTree)) {
                    EXPECT_EQ(non_matching_action_values,0);
                } else {
                    EXPECT_GT(non_matching_action_values,0);
                }
            } else if(backup_type==MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE) {
                EXPECT_EQ(non_matching_action_values,0);
            } else EXPECT_TRUE(false) << "This line should not be reached";
            // visual output
            // if(typeid(*node_finder).name()==typeid(FullGraph).name()) {
            //     search.plot_graph("graph.pdf","neato");
            // } else {
            //     search.plot_graph("graph.pdf","dot");
            // }
            // getchar();
        }
    }
    IF_DEBUG(1) {/* do nothing*/}
    else cout << endl;
}

class MinimalCompleteEnvironment: public IntegerEnvironment {
public:
    MinimalCompleteEnvironment() = default;
    virtual ~MinimalCompleteEnvironment() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(nullptr,action);
        reward_t reward = 0;
        int old_state = state;
        if(state==0) {
            state = 1;
        } else if(state==1) {
            if(action->action==0) {
                if(rand()%3==0) state = 1;
                else state = 2;
            } else if(action->action==1) {
                if(rand()%3==0) state = 2;
                else state = 3;
            } else EXPECT_TRUE(false);
        } else if(state==2) {
            state = 4;
            if(action->action==0) {
                if(rand()%3==0) reward = 1;
                else reward = 0;
            } else if(action->action==1) {
                if(rand()%3==0) reward = 0;
                else reward = 1;
            } else EXPECT_TRUE(false);
        } else if(state==3) {
            state = 4;
            if(action->action==0) {
                if(rand()%3==0) reward = 0;
                else reward = 1;
            } else if(action->action==1) {
                reward = 1;
            } else EXPECT_TRUE(false);
        } else EXPECT_TRUE(false);
        DEBUG_OUT(1,"Transition: (" << old_state << ") " << action->action << " --> " << state << " (" << reward << ")" << (is_terminal_state()?" terminal!":""));
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), reward);
    }
    virtual action_container_t get_actions() override {
        if(state==0) {
            return action_container_t({action_handle_t(new IntegerAction(0))});
        } else {
            return action_container_t({action_handle_t(new IntegerAction(0)),
                        action_handle_t(new IntegerAction(1))});
        }
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state==4;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "MinimalCompleteEnvironment";}
};

TEST(MonteCarloTreeSearch, MinimalCompleteEnvironment) {
    // This test compares the values computed via MCTS to the analytical values.
    // We have to make a trade-off between running time and reliability.
    // Examples: 10000 rollouts with a tolerance of 0.15 in the value (pretty
    // reliable but slow), or 5000 rollouts with a tolerance of 0.2 in the value
    // (acceptable compromise in both respects for catching the worst bugs).
    int rollout_n = 5000;
    double tolerance = 0.2;

    // initialize environment and search tree
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    for(auto node_finder : {
            std::shared_ptr<NodeFinder>(new PlainTree()),
                std::shared_ptr<NodeFinder>(new ObservationTree()),
                std::shared_ptr<NodeFinder>(new FullDAG()),
                std::shared_ptr<NodeFinder>(new FullGraph())
                }) {
        for(auto tree_policy: {
                std::shared_ptr<TreePolicy>(new Uniform()),
                    std::shared_ptr<TreePolicy>(new UCB1()),
                    std::shared_ptr<TreePolicy>(new UCB_Variance(20))
                    }) {
            for(auto value_heuristic : {std::shared_ptr<ValueHeuristic>(new RolloutStatistics(1))}) {
                for(auto backup_method : {
                        std::shared_ptr<BackupMethod>(new Bellman(nullptr,1)),
                            std::shared_ptr<BackupMethod>(new MonteCarlo(1))
                            }) {
                    for(auto backup_type : {
                            MonteCarloTreeSearch::BACKUP_TYPE::TRACE,
                            MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE
                                }) {
                        for(auto rollout_storage : {
                                MonteCarloTreeSearch::ROLLOUT_STORAGE::NONE,
                                    MonteCarloTreeSearch::ROLLOUT_STORAGE::CONDENSED,
                                    MonteCarloTreeSearch::ROLLOUT_STORAGE::FULL
                                    }) {
                            std::stringstream case_string;
                            case_string << "Testing" << endl;
                            case_string << "    " << typeid(*node_finder).name() << endl;
                            case_string << "    " << typeid(*tree_policy).name() << endl;
                            case_string << "    " << typeid(*value_heuristic).name() << endl;
                            case_string << "    " << typeid(*backup_method).name() << endl;
                            case_string << "    " << typeid(backup_type).name() << ": " << (int)backup_type << endl;
                            case_string << "    " << typeid(rollout_storage).name() << ": " << (int)rollout_storage << endl;
                            IF_DEBUG(1) {
                                cout << case_string.str();
                            } else {
                                cout << "." << std::flush;
                            }
                            // things that don't work together
                            if(typeid(*tree_policy)==typeid(Uniform) && typeid(*backup_method)==typeid(MonteCarlo)) {
                                DEBUG_OUT(1,"    Skipping");
                                continue;
                            }
                            // setup everything
                            auto environment = std::shared_ptr<AbstractEnvironment>(new MinimalCompleteEnvironment());
                            double discount = 0.5;
                            MonteCarloTreeSearch search(environment,
                                                        discount,
                                                        node_finder,
                                                        tree_policy,
                                                        value_heuristic,
                                                        backup_method,
                                                        backup_type);
                            search.rollout_storage = rollout_storage;
                            search.data_backups(false);
                            // do rollouts
                            repeat(rollout_n) {
                                search.next();
                                // search.plot_graph("graph.pdf");
                                // getchar();
                            }
                            // checks
                            typedef MonteCarloTreeSearch::graph_t graph_t;
                            typedef MonteCarloTreeSearch::node_t node_t;
                            typedef MonteCarloTreeSearch::node_it_t node_it_t;
                            typedef MonteCarloTreeSearch::arc_t arc_t;
                            typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
                            typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
                            typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
                            const graph_t & graph = search.get_graph();
                            auto & node_info_map = search.get_node_info_map();
                            auto & mcts_node_info_map = search.get_mcts_node_info_map();
                            // root node
                            node_t root_node = INVALID;
                            for(node_it_t node(graph); node!=INVALID; ++node) {
                                if(in_arc_it_t(graph,node)==INVALID) {
                                    root_node = node;
                                    break;
                                }
                            }
                            EXPECT_NE(root_node,INVALID) << case_string.str();
                            // state 1
                            node_t state_1 = graph.target(out_arc_it_t(graph,graph.target(out_arc_it_t(graph,root_node))));
                            EXPECT_EQ(*(node_info_map[state_1].observation),MinimalCompleteEnvironment::IntegerObservation(1)) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[state_1].value,0.44,tolerance) << case_string.str();
                            node_t action_0_from_state_1 = INVALID;
                            node_t action_1_from_state_1 = INVALID;
                            for(out_arc_it_t arc(graph,state_1); arc!=INVALID; ++arc) {
                                node_t action_node = graph.target(arc);
                                if(*(node_info_map[action_node].action)==MinimalCompleteEnvironment::IntegerAction(0)) {
                                    action_0_from_state_1 = action_node;
                                } else if(*(node_info_map[action_node].action)==MinimalCompleteEnvironment::IntegerAction(1)) {
                                    action_1_from_state_1 = action_node;
                                }
                            }
                            EXPECT_NE(action_0_from_state_1,INVALID) << case_string.str();
                            EXPECT_NE(action_1_from_state_1,INVALID) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[action_0_from_state_1].value,0.3,tolerance) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[action_1_from_state_1].value,0.44,tolerance) << case_string.str();
                            // state 2, state 3
                            node_t state_2 = INVALID;
                            node_t state_3 = INVALID;
                            for(out_arc_it_t arc(graph,action_1_from_state_1); arc!=INVALID; ++arc) {
                                node_t state_node = graph.target(arc);
                                if(*(node_info_map[state_node].observation)==MinimalCompleteEnvironment::IntegerObservation(2)) {
                                    state_2 = state_node;
                                } else if(*(node_info_map[state_node].observation)==MinimalCompleteEnvironment::IntegerObservation(3)) {
                                    state_3 = state_node;
                                }
                            }
                            EXPECT_NE(state_2,INVALID) << case_string.str();
                            EXPECT_NE(state_3,INVALID) << case_string.str();
                            EXPECT_EQ(*(node_info_map[state_2].observation),MinimalCompleteEnvironment::IntegerObservation(2)) << case_string.str();
                            EXPECT_EQ(*(node_info_map[state_3].observation),MinimalCompleteEnvironment::IntegerObservation(3)) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[state_2].value,0.67,tolerance) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[state_3].value,1,tolerance) << case_string.str();
                            node_t action_0_from_state_2 = INVALID;
                            node_t action_1_from_state_2 = INVALID;
                            for(out_arc_it_t arc(graph,state_2); arc!=INVALID; ++arc) {
                                node_t action_node = graph.target(arc);
                                if(*(node_info_map[action_node].action)==MinimalCompleteEnvironment::IntegerAction(0)) {
                                    action_0_from_state_2 = action_node;
                                } else if(*(node_info_map[action_node].action)==MinimalCompleteEnvironment::IntegerAction(1)) {
                                    action_1_from_state_2 = action_node;
                                }
                            }
                            EXPECT_NE(action_0_from_state_2,INVALID) << case_string.str();
                            EXPECT_NE(action_1_from_state_2,INVALID) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[action_0_from_state_2].value,0.33,tolerance) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[action_1_from_state_2].value,0.67,tolerance) << case_string.str();
                            node_t action_0_from_state_3 = INVALID;
                            node_t action_1_from_state_3 = INVALID;
                            for(out_arc_it_t arc(graph,state_3); arc!=INVALID; ++arc) {
                                node_t action_node = graph.target(arc);
                                if(*(node_info_map[action_node].action)==MinimalCompleteEnvironment::IntegerAction(0)) {
                                    action_0_from_state_3 = action_node;
                                } else if(*(node_info_map[action_node].action)==MinimalCompleteEnvironment::IntegerAction(1)) {
                                    action_1_from_state_3 = action_node;
                                }
                            }
                            EXPECT_NE(action_0_from_state_3,INVALID) << case_string.str();
                            EXPECT_NE(action_1_from_state_3,INVALID) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[action_0_from_state_3].value,0.67,tolerance) << case_string.str();
                            EXPECT_NEAR(mcts_node_info_map[action_1_from_state_3].value,1,tolerance) << case_string.str();
                            // search.plot_graph("graph.pdf");
                            // getchar();
                        }
                    }
                }
            }
        }
    }
    IF_DEBUG(1) {/* do nothing*/}
    else cout << endl;
}

/** This environment is designed to check variance updates. The transition and
 * reward tables are as follows
 *
 * \f[
 \begin{align}
  \begin{array}{c|cc|c|c|}
    \text{state (from)} & 0 & & 1 & 2 \\
    \hline
    \text{action} & 0 & 1 & 2 & 3 \\
    \hline\hline
    1 & 1 & 0 & 0 & 0 \\
    2 & 0 & 1 & 0 & 0 \\
    3 & 0 & 0 & 0.5 & 0 \\
    4 & 0 & 0 & 0.5 & 0 \\
    5 & 0 & 0 & 0 & 0.5 \\
    6 & 0 & 0 & 0 & 0.5 \\
    \hline
  \end{array}
  \qquad
  \begin{array}{c|c|c|c|c|}
    \text{reached state} & 3 & 4 & 5 & 6 \\
    \hline\hline
    0 & 0 & 0 & 0.5 & 0.5 \\
    \frac{1}{10} & 1 & 0 & 0 & 0 \\
    \frac{9}{10} & 0 & 1 & 0 & 0 \\
    1 & 0 & 0 & 0.5 & 0.5 \\
    \hline
  \end{array}
\end{align}
 * \f]
 * */
class MinimalVarianceCheckEnvironment: public IntegerEnvironment {
public:
    int last_terminal_state_from_1 = 0;
    int last_terminal_state_from_2 = 0;
    int last_reward_in_state_5 = 0;
    int last_reward_in_state_6 = 0;
    MinimalVarianceCheckEnvironment() = default;
    virtual ~MinimalVarianceCheckEnvironment() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(nullptr,action);
        EXPECT_TRUE(state==0 || state==1 || state==2);
        reward_t reward = 0;
        switch(state) {
        case 0:
            EXPECT_TRUE(action->action==0 || action->action==1);
            if(action->action==0) {
                state = 1;
            } else {
                state = 2;
            }
            break;
        case 1:
            EXPECT_EQ(action->action,2);
            if(last_terminal_state_from_1==3) {
                state = 4;
                reward = 9./10;
            } else {
                state = 3;
                reward = 1./10;
            }
            last_terminal_state_from_1 = state;
            break;
        case 2:
            EXPECT_EQ(action->action,3);
            if(last_terminal_state_from_2==5) {
                state = 6;
                if(last_reward_in_state_6==0) reward = 1;
                else reward = 0;
                last_reward_in_state_6 = reward;
            } else {
                state = 5;
                if(last_reward_in_state_5==0) reward = 1;
                else reward = 0;
                last_reward_in_state_5 = reward;
            }
            last_terminal_state_from_2 = state;
            break;
        default:
            EXPECT_TRUE(false) << "This line should never be reached";
        }
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), reward);
    }
    virtual action_container_t get_actions() override {
        switch(state) {
        case 0:
            return action_container_t({action_handle_t(new IntegerAction(0)),
                        action_handle_t(new IntegerAction(1))});
        case 1:
            return action_container_t({action_handle_t(new IntegerAction(2))});
        case 2:
            return action_container_t({action_handle_t(new IntegerAction(3))});
        default:
            return action_container_t({});
        }
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state>2;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "MinimalVarianceCheckEnvironment";}
};

TEST(MonteCarloTreeSearch, VarianceBackups) {
    // This test compares analytical variance value to those computed via MCTS
    int rollout_n = 8*100;
    double tolerance = 1e-10;

    // initialize environment and search tree
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    auto bellman_backup_policy = std::shared_ptr<TreePolicy>(new Uniform());
    for(auto backup_method : {
            std::shared_ptr<BackupMethod>(new Bellman(bellman_backup_policy,0)),
                std::shared_ptr<BackupMethod>(new MonteCarlo(0))
                }) {
        auto node_finder = std::shared_ptr<NodeFinder>(new PlainTree());
        auto tree_policy = std::shared_ptr<TreePolicy>(new UCB1(1e10));
        auto value_heuristic = std::shared_ptr<ValueHeuristic>(new RolloutStatistics(0));
        auto backup_type = MonteCarloTreeSearch::BACKUP_TYPE::TRACE;
        auto rollout_storage = MonteCarloTreeSearch::ROLLOUT_STORAGE::CONDENSED; // NONE/CONDENSED/FULL
        std::stringstream case_string;
        case_string << "Testing" << endl;
        case_string << "    " << typeid(*node_finder).name() << endl;
        case_string << "    " << typeid(*tree_policy).name() << endl;
        case_string << "    " << typeid(*value_heuristic).name() << endl;
        case_string << "    " << typeid(*backup_method).name() << endl;
        case_string << "    " << typeid(backup_type).name() << ": " << (int)backup_type << endl;
        case_string << "    " << typeid(rollout_storage).name() << ": " << (int)rollout_storage << endl;
        IF_DEBUG(1) {
            cout << case_string.str();
        } else {
            cout << "." << std::flush;
        }
        // setup everything
        auto environment = std::shared_ptr<AbstractEnvironment>(new MinimalVarianceCheckEnvironment());
        double discount = 1;
        MonteCarloTreeSearch search(environment,
                                    discount,
                                    node_finder,
                                    tree_policy,
                                    value_heuristic,
                                    backup_method,
                                    backup_type);
        search.rollout_storage = rollout_storage;
        search.data_backups(false);
        // for the checks
        typedef MonteCarloTreeSearch::graph_t graph_t;
        typedef MonteCarloTreeSearch::node_t node_t;
        typedef MonteCarloTreeSearch::node_it_t node_it_t;
        typedef MonteCarloTreeSearch::arc_t arc_t;
        typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
        typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
        typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
        const graph_t & graph = search.get_graph();
        auto & node_info_map = search.get_node_info_map();
        auto & mcts_node_info_map = search.get_mcts_node_info_map();
        // do rollouts
        for(int rollout_count=1; rollout_count<=rollout_n; ++rollout_count) {
            search.next();
            if(rollout_count>=6 && rollout_count%8==0) {
                //search.plot_graph("graph.pdf");
                int n = rollout_count/8;
                bool is_bellman = typeid(*backup_method)==typeid(Bellman);
                for(node_it_t node(graph); node!=INVALID; ++node) {
                    if(node_info_map[node].type==MonteCarloTreeSearch::ACTION_NODE) {
                        auto action = std::dynamic_pointer_cast<const MinimalVarianceCheckEnvironment::IntegerAction>(node_info_map[node].action);
                        EXPECT_NE(action,nullptr);
                        switch(action->action) {
                        case 0:
                            EXPECT_NEAR(mcts_node_info_map[node].value,0.5,tolerance) << "Value for action " << action->action << " does not match";
                            if(is_bellman) EXPECT_NEAR(mcts_node_info_map[node].value_variance,4./(25*(4*n+1)),tolerance) << "Variance for action " << action->action << " does not match";
                            else EXPECT_NEAR(mcts_node_info_map[node].value_variance,4./(25*(4*n-1)),tolerance) << "Variance for action " << action->action << " does not match";
                            break;
                        case 1:
                            EXPECT_NEAR(mcts_node_info_map[node].value,0.5,tolerance) << "Value for action " << action->action << " does not match";
                            if(is_bellman) EXPECT_NEAR(mcts_node_info_map[node].value_variance,(2.*n+1.)/(4*(8*n*n-2*n-1)),tolerance) << "Variance for action " << action->action << " does not match";
                            else EXPECT_NEAR(mcts_node_info_map[node].value_variance,1./(4*(4*n-1)),tolerance) << "Variance for action " << action->action << " does not match";
                            break;
                        case 2:
                            EXPECT_NEAR(mcts_node_info_map[node].value,0.5,tolerance) << "Value for action " << action->action << " does not match";
                            if(is_bellman) EXPECT_NEAR(mcts_node_info_map[node].value_variance,4./(25*(4*n+1)),tolerance) << "Variance for action " << action->action << " does not match";
                            else EXPECT_NEAR(mcts_node_info_map[node].value_variance,4./(25*(4*n-1)),tolerance) << "Variance for action " << action->action << " does not match";
                            break;
                        case 3:
                            EXPECT_NEAR(mcts_node_info_map[node].value,0.5,tolerance) << "Value for action " << action->action << " does not match";
                            if(is_bellman) EXPECT_NEAR(mcts_node_info_map[node].value_variance,(2.*n+1.)/(4*(8*n*n-2*n-1)),tolerance) << "Variance for action " << action->action << " does not match";
                            else EXPECT_NEAR(mcts_node_info_map[node].value_variance,1./(4*(4*n-1)),tolerance) << "Variance for action " << action->action << " does not match";
                            break;
                        default:
                            EXPECT_TRUE(false) << "This line should never be reached";
                            break;
                        }
                    } else if(node_info_map[node].type==MonteCarloTreeSearch::OBSERVATION_NODE) {
                        auto observation = std::dynamic_pointer_cast<const MinimalVarianceCheckEnvironment::IntegerObservation>(node_info_map[node].observation);
                        int state;
                        if(observation==nullptr) {
                            state = 0;
                        } else {
                            state = observation->observation;
                        }
                        switch(state) {
                        case 0:
                            EXPECT_NEAR(mcts_node_info_map[node].value,0.5,tolerance) << "Value for state " << state << " does not match";
                            if(is_bellman) EXPECT_NEAR(mcts_node_info_map[node].value_variance,(82.*n+9.)/(400*(8*n*n-2*n-1)),tolerance) << "Variance for state " << state << " does not match";
                            else EXPECT_NEAR(mcts_node_info_map[node].value_variance,41./(200*(8*n-1)),tolerance) << "Variance for state " << state << " does not match";
                            break;
                        case 1:
                            EXPECT_NEAR(mcts_node_info_map[node].value,0.5,tolerance) << "Value for state " << state << " does not match";
                            if(is_bellman) EXPECT_NEAR(mcts_node_info_map[node].value_variance,4./(25*(4*n+1)),tolerance) << "Variance for state " << state << " does not match";
                            else EXPECT_NEAR(mcts_node_info_map[node].value_variance,4./(25*(4*n-1)),tolerance) << "Variance for state " << state << " does not match";
                            break;
                        case 2:
                            EXPECT_NEAR(mcts_node_info_map[node].value,0.5,tolerance) << "Value for state " << state << " does not match";
                            if(is_bellman) EXPECT_NEAR(mcts_node_info_map[node].value_variance,(2.*n+1.)/(4*(8*n*n-2*n-1)),tolerance) << "Variance for state " << state << " does not match";
                            else EXPECT_NEAR(mcts_node_info_map[node].value_variance,1./(4*(4*n-1)),tolerance) << "Variance for state " << state << " does not match";
                            break;
                        case 3:
                        case 4:
                        case 5:
                        case 6:
                            EXPECT_NEAR(mcts_node_info_map[node].value,0,tolerance) << "Value for state " << state << " does not match";
                            EXPECT_NEAR(mcts_node_info_map[node].value_variance,0,tolerance) << "Variance for state " << state << " does not match";
                            break;
                        default:
                            EXPECT_TRUE(false) << "This line should never be reached";
                            break;
                        }
                    } else EXPECT_TRUE(false) << "This line should never be reached";
                }
                //getchar();
            }
        }
        // search.plot_graph("graph.pdf");
        // getchar();
    }
    IF_DEBUG(1) {/* do nothing*/}
    else cout << endl;
}

#if 0
// This test is supposed to check the variance computed via data backups versus
// "normal" backup. This test fails! But this may be due to the quantities being
// computed differently. I don't know whether a direct comparison can be made.
TEST(MonteCarloTreeSearch, DataBackup) {
    // initialize environment and search tree
    using namespace node_finder;
    using namespace tree_policy;
    using namespace value_heuristic;
    using namespace backup_method;
    for(auto node_finder : {
            std::shared_ptr<NodeFinder>(new PlainTree()),
                std::shared_ptr<NodeFinder>(new ObservationTree()),
                std::shared_ptr<NodeFinder>(new FullDAG()),
                std::shared_ptr<NodeFinder>(new FullGraph())
                }) {
        for(auto tree_policy: {
                std::shared_ptr<TreePolicy>(new Uniform()),
                    std::shared_ptr<TreePolicy>(new UCB1()),
                    std::shared_ptr<TreePolicy>(new UCB_Variance(20))
                    }) {
            for(auto value_heuristic : {std::shared_ptr<ValueHeuristic>(new RolloutStatistics(0))}) {
                for(auto backup_method : {
                        std::shared_ptr<BackupMethod>(new Bellman(nullptr,0)),
                            std::shared_ptr<BackupMethod>(new MonteCarlo(0))
                            }) {
                    for(auto backup_type : {
                            MonteCarloTreeSearch::BACKUP_TYPE::TRACE,
                            MonteCarloTreeSearch::BACKUP_TYPE::PROPAGATE
                                }) {
                        for(auto rollout_storage : {
                                //MonteCarloTreeSearch::ROLLOUT_STORAGE::NONE,
                                    MonteCarloTreeSearch::ROLLOUT_STORAGE::CONDENSED,
                                    MonteCarloTreeSearch::ROLLOUT_STORAGE::FULL
                                    }) {
                            std::stringstream case_string;
                            case_string << "Testing" << endl;
                            case_string << "    " << typeid(*node_finder).name() << endl;
                            case_string << "    " << typeid(*tree_policy).name() << endl;
                            case_string << "    " << typeid(*value_heuristic).name() << endl;
                            case_string << "    " << typeid(*backup_method).name() << endl;
                            case_string << "    " << typeid(backup_type).name() << ": " << (int)backup_type << endl;
                            case_string << "    " << typeid(rollout_storage).name() << ": " << (int)rollout_storage << endl;
                            IF_DEBUG(1) {
                                cout << case_string.str();
                            } else {
                                cout << "." << std::flush;
                            }
                            // things that don't work together
                            // if(typeid(*tree_policy)==typeid(Uniform) && typeid(*backup_method)==typeid(MonteCarlo)) {
                            //     DEBUG_OUT(1,"    Skipping");
                            //     continue;
                            // }
                            // setup everything
                            auto environment = std::shared_ptr<AbstractEnvironment>(new MinimalCompleteEnvironment());
                            double discount = 0.5;
                            MonteCarloTreeSearch search(environment,
                                                        discount,
                                                        node_finder,
                                                        tree_policy,
                                                        value_heuristic,
                                                        backup_method,
                                                        backup_type);
                            search.rollout_storage = rollout_storage;
                            search.data_backups(true);
                            // do rollouts
                            int rollout_n = 100;
                            repeat(rollout_n) {
                                search.next();
                                // search.plot_graph("graph.pdf");
                                // getchar();
                            }
                            //--------//
                            // checks //
                            //--------//
                            typedef MonteCarloTreeSearch::graph_t graph_t;
                            typedef MonteCarloTreeSearch::node_t node_t;
                            typedef MonteCarloTreeSearch::node_it_t node_it_t;
                            typedef MonteCarloTreeSearch::arc_t arc_t;
                            typedef MonteCarloTreeSearch::arc_it_t arc_it_t;
                            typedef MonteCarloTreeSearch::in_arc_it_t in_arc_it_t;
                            typedef MonteCarloTreeSearch::out_arc_it_t out_arc_it_t;
                            const graph_t & graph = search.get_graph();
                            auto & node_info_map = search.get_node_info_map();
                            auto & mcts_node_info_map = search.get_mcts_node_info_map();
                            // find nodes with fully expanded sub-trees by
                            // "reverse flooding" from not-fully-expanded nodes
                            typedef lemon::ReverseDigraph<const graph_t> const_graph_t;
                            const_graph_t reverse_graph = lemon::reverseDigraph(graph);
                            graph_t::NodeMap<bool> reached(graph);
                            auto flooding = graph_util::GraphFlooding<const_graph_t,graph_t>(reverse_graph,reached);
                            for(node_it_t node(graph); node!=INVALID; ++node) {
                                if(mcts_node_info_map[node].rollout_counts<2 || // for valid variance
                                   mcts_node_info_map[node].rollout_counts!=(int)mcts_node_info_map[node].rollout_set.size()) {
                                    flooding.add_source(node);
                                    DEBUG_OUT(2,"    node " << graph.id(node) << " not fully expanded");
                                }
                            }
                            flooding.flood();
                            // now check nodes that were not reached, i.e., that
                            // have fully expanded subtrees
                            for(node_it_t node(graph); node!=INVALID; ++node) {
                                if(reached[node]) continue;
                                DEBUG_OUT(2,"    checking node " << graph.id(node));
                                auto & rollout_set = mcts_node_info_map[node].rollout_set;
                                double mean_return = 0;
                                double mean_return_square = 0;
                                double counts = 0;
                                double weight_sum = 0;
                                for(auto & rollout_item : rollout_set) {
                                    mean_return += rollout_item->weight * rollout_item->discounted_return;
                                    mean_return_square += rollout_item->weight * pow(rollout_item->discounted_return,2);
                                    weight_sum += rollout_item->weight;
                                    ++counts;
                                }
                                double return_variance = (counts/(counts-1))*(mean_return_square-pow(mean_return,2));
                                double value_variance = return_variance/counts;

                                DEBUG_WARNING("mean return: " << mean_return);
                                DEBUG_WARNING("mean return square: " << mean_return_square);
                                DEBUG_WARNING("counts: " << counts);
                                DEBUG_WARNING("return variance: " << return_variance);
                                DEBUG_WARNING("value variance: " << value_variance);
                                DEBUG_WARNING("value variance [node]: " << mcts_node_info_map[node].value_variance);
                                DEBUG_WARNING("value [node]: " << mcts_node_info_map[node].value);

                                DEBUG_EXPECT_APPROX(weight_sum,1);
                                DEBUG_EXPECT_APPROX(mean_return,mcts_node_info_map[node].value);
                                DEBUG_EXPECT_APPROX(value_variance,mcts_node_info_map[node].value_variance);

                                if(fabs(mean_return-mcts_node_info_map[node].value)>1e-10 ||
                                   fabs(value_variance-mcts_node_info_map[node].value_variance)>1e-10) {
                                    for(auto & rollout_item : rollout_set) {
                                        cout << *rollout_item << endl;
                                    }
                                }
                            }

                            search.plot_graph("graph.pdf");
                            getchar();
                        }
                    }
                }
            }
        }
    }
    IF_DEBUG(1) {/* do nothing*/}
    else cout << endl;
}
#endif

class StochasticFiniteLineEnvironment: public IntegerEnvironment {
public:
    StochasticFiniteLineEnvironment(int line_width): line_width(line_width) {}
    virtual ~StochasticFiniteLineEnvironment() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto line_action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        EXPECT_NE(nullptr,line_action);
        //state += (line_action->action>0?1:0);
        if(rand()%2==0) {
            // only change state in 50% of the cases
            state += line_action->action;
        }
        if(state>line_width) state = line_width;
        if(state<0) state = 0;
        reward_t reward = state==line_width?1:0;
        DEBUG_OUT(1,"Transition to " << state);
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)), reward);
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new IntegerAction(-1)),
                    action_handle_t(new IntegerAction(0)),
                    action_handle_t(new IntegerAction(1))});
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state==line_width;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {out << "StochasticFiniteLineEnvironment";}
private:
    int line_width;
};

/**
 * Special derived class for unit tests. The computations in this version simply
 * sum up all incomming values for any computational node. The values are
 * initialized to 1 for independent variables and 0 for dependent variables
 * (alpha, beta, gamma, A, B, C are considered dependent variables even if they
 * currently happen to have no incoming arc because they correspond to
 * leaf-nodes). In this setting each independen variables contributes to the
 * value of every dependent variable (including the root node) once via every
 * possible path. This allows for a simple check of values and derivaties. */
class MockActiveTreeSearch: public ActiveTreeSearch {
public:
    MockActiveTreeSearch(std::shared_ptr<AbstractEnvironment> environment,
                         double discount,
                         std::shared_ptr<node_finder::NodeFinder> node_finder):
        ActiveTreeSearch(environment,discount,node_finder)
    {}
    virtual ~MockActiveTreeSearch() = default;
    const graph_t & get_c_graph() const {return c_graph;}
    const ComputationalConstGraph & get_computer() const {return computer;}
    virtual void update_c_node_connections(node_t action_node) override {
        DEBUG_EXPECT(node_info_map[action_node].type==ACTION_NODE);

        for(node_t c_node : {
                variable_info_map[action_node].pi,
                    variable_info_map[action_node].mean_Q,
                    variable_info_map[action_node].var_Q,
                    variable_info_map[action_node].mean_r,
                    variable_info_map[action_node].var_r,
                    variable_info_map[action_node].A,
                    variable_info_map[action_node].B,
                    variable_info_map[action_node].C
                    }) {
            vector<QString> input_names;
            for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                input_names.push_back(computer.get_node_label(c_graph.source(arc)));
            }
            computer.set_node_function(c_node,
                                       input_names,
                                       [](vector<double> v)->double{
                                           double ret = 0;
                                           for(double val : v) ret += val;
                                           return ret;
                                       });
            for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                computer.set_arc(arc, [](vector<double>)->double{return 1;});
            }
        }

        for(node_array_t array : {
                variable_info_map[action_node].mean_p,
                    variable_info_map[action_node].alpha,
                    variable_info_map[action_node].beta
                    }) {
            for(auto c_node_pair : array) {
                node_t c_node = c_node_pair.second;
                vector<QString> input_names;
                for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                    input_names.push_back(computer.get_node_label(c_graph.source(arc)));
                }
                computer.set_node_function(c_node,
                                           input_names,
                                           [](vector<double> v)->double{
                                               double ret = 0;
                                               for(double val : v) ret += val;
                                               return ret;
                                           });
                for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                    computer.set_arc(arc, [](vector<double>)->double{return 1;});
                }
            }
        }

        for(node_matrix_t matrix : {
                variable_info_map[action_node].var_p,
                    variable_info_map[action_node].gamma
                    }) {
            for(auto array : matrix) {
                for(auto c_node_pair : array.second) {
                    node_t c_node = c_node_pair.second;
                    vector<QString> input_names;
                    for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                        input_names.push_back(computer.get_node_label(c_graph.source(arc)));
                    }
                    computer.set_node_function(c_node,
                                               input_names,
                                               [](vector<double> v)->double{
                                                   double ret = 0;
                                                   for(double val : v) ret += val;
                                                   return ret;
                                               });
                    for(in_arc_it_t arc(c_graph,c_node); arc!=INVALID; ++arc) {
                        computer.set_arc(arc, [](vector<double>)->double{return 1;});
                    }
                }
            }
        }

        // update values (which were invalidated above)
        update_c_node_values(action_node);
    }
    virtual void update_c_node_values(node_t action_node) override {
        // independent variables
        for(node_t c_node : {
                variable_info_map[action_node].mean_r,
                    variable_info_map[action_node].var_r,
                    }) {
            computer.set_node_value(c_node,1);
        }

        for(node_array_t array : {
                variable_info_map[action_node].mean_p,
                    }) {
            for(auto c_node_pair : array) {
                node_t c_node = c_node_pair.second;
                computer.set_node_value(c_node,1);
            }
        }
        for(node_matrix_t matrix : {
                variable_info_map[action_node].var_p,
                    }) {
            for(auto array : matrix) {
                for(auto c_node_pair : array.second) {
                    node_t c_node = c_node_pair.second;
                    computer.set_node_value(c_node,1);
                }
            }
        }

        // dependent variables
        for(node_t c_node : {
                variable_info_map[action_node].pi,
                    variable_info_map[action_node].mean_Q,
                    variable_info_map[action_node].var_Q,
                    variable_info_map[action_node].A,
                    variable_info_map[action_node].B,
                    variable_info_map[action_node].C
                    }) {
            // set to zero if no input available
            if(in_arc_it_t(c_graph,c_node)==INVALID) computer.set_node_value(c_node,0);
        }

        for(node_array_t array : {
                variable_info_map[action_node].alpha,
                    variable_info_map[action_node].beta
                    }) {
            for(auto c_node_pair : array) {
                node_t c_node = c_node_pair.second;
                // set to zero if no input available
                if(in_arc_it_t(c_graph,c_node)==INVALID) computer.set_node_value(c_node,0);
            }
        }
        for(node_matrix_t matrix : {
                variable_info_map[action_node].gamma
                    }) {
            for(auto array : matrix) {
                for(auto c_node_pair : array.second) {
                    node_t c_node = c_node_pair.second;
                    // set to zero if no input available
                    if(in_arc_it_t(c_graph,c_node)==INVALID) computer.set_node_value(c_node,0);
                }
            }
        }
    }
    virtual void update_c_root_connections() override {
        vector<QString> input_names;
        for(in_arc_it_t arc(c_graph,c_root_node); arc!=INVALID; ++arc) {
            input_names.push_back(computer.get_node_label(c_graph.source(arc)));
        }
        computer.set_node_function(c_root_node,
                                   input_names,
                                   [](vector<double> v)->double{
                                       double ret = 0;
                                       for(double val : v) ret += val;
                                       return ret;
                                   });
        for(in_arc_it_t arc(c_graph,c_root_node); arc!=INVALID; ++arc) {
            computer.set_arc(arc, [](vector<double>)->double{return 1;});
        }
    }
};

TEST(ActiveTreeSearch, SimpleValueCheck) {
    auto environment = std::shared_ptr<AbstractEnvironment>(new StochasticFiniteLineEnvironment(2));
    MockActiveTreeSearch search(environment,
                                1,
                                std::shared_ptr<node_finder::NodeFinder>(new node_finder::PlainTree()));
    for(int i=0; i<10; ++i) {
        search.next();
        // visual output
        // search.toPdf("graph.pdf");
        // getchar();
    }
    // checks
    typedef MockActiveTreeSearch::graph_t graph_t;
    typedef MockActiveTreeSearch::node_t node_t;
    typedef MockActiveTreeSearch::node_it_t node_it_t;
    typedef MockActiveTreeSearch::arc_t arc_t;
    typedef MockActiveTreeSearch::arc_it_t arc_it_t;
    typedef MockActiveTreeSearch::in_arc_it_t in_arc_it_t;
    typedef MockActiveTreeSearch::out_arc_it_t out_arc_it_t;
    auto & graph = search.get_c_graph();
    auto & computer = search.get_computer();
    graph_t::NodeMap<int> value_map(graph,0);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        // currently explored path
        vector<pair<node_t,out_arc_it_t>> path;
        // follow-path function
        auto follow_path = [&](){
            while(path.back().second!=INVALID) {
                node_t new_node = graph.target(path.back().second);
                path.push_back(make_pair(new_node,out_arc_it_t(graph,new_node)));
                if(in_arc_it_t(graph,node)==INVALID && computer.get_node_value(node)==1) {
                    ++value_map[new_node];
                }
            }
        };
        // backtracking function
        auto backtrack = [&]() {
            // we should be at a (the one and only actually) terminal node
            EXPECT_EQ(path.back().second,INVALID);
            while(path.back().second==INVALID) {
                // remove from path
                path.pop_back();
                // in case we removed the initial node there are no more paths
                // to discover
                if(path.empty()) return;
                // now we should be at a non-terminal node
                EXPECT_NE(path.back().second,INVALID);
                // iterate to next outgoing arc
                ++path.back().second;
            }
        };
        // initialize with current node and follow to terminal node
        path.push_back(make_pair(node,out_arc_it_t(graph,node)));
        int path_counter = 0;
        while(!path.empty()) {
            follow_path();
            backtrack();
            ++path_counter;
        }
        // the differential should be equal to the number of paths (to the root
        // node)
        EXPECT_EQ(computer.get_node_differential(node),path_counter);
    }
    // check value (must be the number of paths from independen variables
    // passing that node)
    for(node_it_t node(graph); node!=INVALID; ++node) {
        // only internal nodes
        if(in_arc_it_t(graph,node)!=INVALID)
            EXPECT_EQ(computer.get_node_value(node),value_map[node]);
    }
}

TEST(ActiveTreeSearch, Test) {
    auto environment = std::shared_ptr<AbstractEnvironment>(new StochasticFiniteLineEnvironment(2));
    ActiveTreeSearch search(environment,
                            1,
                            std::shared_ptr<node_finder::NodeFinder>(new node_finder::PlainTree()));
    for(int i=0; i<30; ++i) {
        search.next();
        // visual output
        // search.toPdf("graph.pdf");
        // getchar();
    }
    //search.toPdf("graph.pdf");
}

TEST(PriorModels, PriorCounts) {
    // remove bias (default)
    {
        prior_models::PriorCounts prior_counts(1,1,2,0,1,2);
        EXPECT_EQ(0.5,prior_models::PriorCounts::compute_mean(1,2,0,1,2));
        EXPECT_EQ(0.5,prior_counts.mean);
        EXPECT_EQ(0.25*4./3.,prior_models::PriorCounts::compute_variance(0.5,1,2,0,1,2));
        EXPECT_EQ(0.25*4./3.,prior_counts.variance);
        EXPECT_EQ(0.25*4./3./(2+2),prior_counts.variance_of_mean);
    }
    // don't remove bias
    {
        prior_models::PriorCounts prior_counts(1,1,2,0,1,2,false);
        EXPECT_EQ(0.5,prior_models::PriorCounts::compute_mean(1,2,0,1,2));
        EXPECT_EQ(0.5,prior_counts.mean);
        EXPECT_EQ(0.25,prior_models::PriorCounts::compute_variance(0.5,1,2,0,1,2,false));
        EXPECT_EQ(0.25,prior_counts.variance);
        EXPECT_EQ(0.25/4,prior_counts.variance_of_mean);
    }
}

TEST(PriorModels, Dirichlet) {
    // uniform counts
    {
        double n = 10;
        prior_models::Dirichlet dirichlet(vector<double>(n,1));
        double norm = n*n*(n+1);
        for(int idx_1=0; idx_1<n; ++idx_1) {
            EXPECT_EQ(dirichlet.mean[idx_1],1/n);
            for(int idx_2=0; idx_2<n; ++idx_2) {
                if(idx_1==idx_2) {
                    EXPECT_EQ(dirichlet.covariance[idx_1][idx_2],(n-1)/norm);
                } else {
                    EXPECT_EQ(dirichlet.covariance[idx_1][idx_2],-1/(norm));
                }
            }
        }
    }
    // strongly non-uniform counts
    {
        double n = 10;
        double high = 100;
        double counts = n-1+high;
        vector<double> count_vec(n,1);
        count_vec[0] = high;
        prior_models::Dirichlet dirichlet(count_vec);
        double norm = (counts)*(counts)*(counts+1);
        for(int idx_1=0; idx_1<n; ++idx_1) {
            if(idx_1==0) {
                EXPECT_EQ(dirichlet.mean[idx_1],high/counts);
            } else {
                EXPECT_EQ(dirichlet.mean[idx_1],1/counts);
            }
            for(int idx_2=0; idx_2<n; ++idx_2) {
                if(idx_1==0 && idx_2==0) {
                    EXPECT_EQ(dirichlet.covariance[idx_1][idx_2],high*(counts-high)/norm);
                } else if(idx_1==0 || idx_2==0) {
                    EXPECT_EQ(dirichlet.covariance[idx_1][idx_2],-high/norm);
                } else if(idx_1==idx_2) {
                    EXPECT_EQ(dirichlet.covariance[idx_1][idx_2],(counts-1)/norm);
                } else {
                    EXPECT_EQ(dirichlet.covariance[idx_1][idx_2],-1/norm);
                }
            }
        }
    }
}
