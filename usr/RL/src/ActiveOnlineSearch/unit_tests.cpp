/** @file
 * \brief This file implements unit tests for the ActiveOnlineSearch
 * library and executable. */

#include <gtest/gtest.h>

#include "../util/pretty_printer.h"
#include "../util/util.h"
#include "../util/ND_vector.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

#include <ActiveOnlineSearch/ComputationalGraph.h>

using namespace ND_vector;
using util::Range;
using std::vector;
using std::cout;
using std::endl;

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

    graph_t graph;
    ComputationalGraph cg(graph);

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
