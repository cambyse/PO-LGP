#include <gtest/gtest.h>

#include "../util/util.h"
#include "../util/ND_vector.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

#include <ActiveOnlineSearch/ComputationalGraph.h>

using namespace ND_vector;
using util::Range;
using std::vector;

TEST(ActiveOnlineSearch, ComputationalGraph) {

    /*=============================================
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
    /*=============================================*/


    typedef ComputationalGraph::node_t node_t;
    typedef lemon::ListDigraph graph_t;

    auto graph = std::make_shared<graph_t>();
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

    // fill lists of input/output nodes
    cg.set_input_nodes({alpha_node, w_node, t_node});
    cg.set_output_nodes({x_node, y_node});

    // check graph structure and in-/output nodes
    {
        bool ok;
        EXPECT_TRUE(ok=cg.check_graph_structure()) <<
            "Aborting tests to avoid initinte loop during computation";
        if(!ok) return;
    }
    // check derivatives
    EXPECT_TRUE(cg.check_derivatives({0.1,3,30}));

    // compute values and check
    cg.compute_values({0.1,3,30});
    vec_double_1D output_vals = cg.get_output_values();
    vec_double_1D analytical_output_vals({x(0.1,3,30),y(0.1,3,30)});
    EXPECT_EQ(output_vals, analytical_output_vals);

    // compute derivatives via forward accumulation
    cg.forward_accumulation({0.1,3,30},{1,0,0});
    vec_double_1D output_diff_1 = cg.get_output_differentials();
    cg.forward_accumulation({0.1,3,30},{0,1,0});
    vec_double_1D output_diff_2 = cg.get_output_differentials();
    cg.forward_accumulation({0.1,3,30},{0,0,1});
    vec_double_1D output_diff_3 = cg.get_output_differentials();

    // compute derivatives via reverse accumulation
    cg.reverse_accumulation({0.1,3,30},{1,0});
    vec_double_1D input_diff_1 = cg.get_input_differentials();
    cg.reverse_accumulation({0.1,3,30},{0,1});
    vec_double_1D input_diff_2 = cg.get_input_differentials();

    // construct matrices of patrial derivatives
    vec_double_2D input_output_diff({output_diff_1, output_diff_2, output_diff_3});
    vec_double_2D output_input_diff({input_diff_1, input_diff_2});
    vec_double_2D analytical_input_output_diff({
            {dx_dalpha(0.1,3,30), dy_dalpha(0.1,3,30)},
            {dx_dw(0.1,3,30),     dy_dw(0.1,3,30)},
            {dx_dt(0.1,3,30),     dy_dt(0.1,3,30)}});


    for(int input_idx : Range(3)) {
        for(int output_idx : Range(2)) {
            // compensate for numerical rounding errors, which might be
            // different in forward and reverse propagation
            EXPECT_NEAR(input_output_diff[input_idx][output_idx],
                        output_input_diff[output_idx][input_idx], 1e-10);
            EXPECT_NEAR(analytical_input_output_diff[input_idx][output_idx],
                        output_input_diff[output_idx][input_idx], 1e-10);
        }
    }
}
