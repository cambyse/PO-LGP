#include <gtest/gtest.h>

#include "../util/ColorOutput.h"
#include "../optimization/LBFGS_Object.h"

#include <math.h>

#define DEBUG_LEVEL 1
#include "../debug.h"

using std::function;

TEST(LBFGSTest, NoObjective) {
    LBFGS_Object o;
    o.optimize();
}

// objective
double f(const double* x, double* g) {
    // f(x0,x1) = x0^2/10 + x1^2/10 + (x0*x1+2)^2
    // df/dx0 = 2x0/10 + 2(x0*x1+2)(x1)
    // df/dx1 = 2x1/10 + 2(x0*x1+2)(x0)
    // minima: (+sqrt(19/10),-sqrt(19/10)), (-sqrt(19/10),+sqrt(19/10))
    g[0] = 2.*x[0]/10 + 2*(x[0]*x[1]+2)*x[1];
    g[1] = 2.*x[1]/10 + 2*(x[0]*x[1]+2)*x[0];
    return pow(x[0],2)/10 + pow(x[1],2)/10 + pow(x[0]*x[1]+2,2);
}

TEST(LBFGSTest, CallWithArray) {
    LBFGS_Object o;
    double init[2] = {10,-10};
    o.set_number_of_variables(2);
    o.optimize(LBFGS_Object::objective_t(&f),init);
    EXPECT_NEAR( sqrt(19./10),init[0],0.001);
    EXPECT_NEAR(-sqrt(19./10),init[1],0.001);
}

TEST(LBFGSTest, CallWithVector) {
    LBFGS_Object o;
    std::vector<double> init = {-10,10};
    o.set_objective(LBFGS_Object::objective_t(&f));
    o.optimize(init);
    EXPECT_NEAR(-sqrt(19./10),init[0],0.001);
    EXPECT_NEAR( sqrt(19./10),init[1],0.001);
}

int p (const lbfgsfloatval_t */*x*/,
       const lbfgsfloatval_t */*g*/,
       const lbfgsfloatval_t /*fx*/,
       const lbfgsfloatval_t /*xnorm*/,
       const lbfgsfloatval_t /*gnorm*/,
       const lbfgsfloatval_t /*step*/,
       int nr_variables,
       int iteration_nr,
       int /*ls*/) {
    DEBUG_OUT(1, "Print progress for " << nr_variables << " variables in iteration " << iteration_nr);
    return 0;
}

TEST(LBFGSTest, CallWithProgress) {
    LBFGS_Object o;
    std::vector<double> init = {-10,10};
    o.set_objective(LBFGS_Object::objective_t(&f));
    o.set_progress(LBFGS_Object::progress_t(&p));
    o.optimize(init);
    EXPECT_NEAR(-sqrt(19./10),init[0],0.001);
    EXPECT_NEAR( sqrt(19./10),init[1],0.001);
}

TEST(LBFGSTest, CheckDerivatives) {
    LBFGS_Object o;
    o.set_objective(LBFGS_Object::objective_t(&f)).set_number_of_variables(2).check_derivatives(10,5);
}
