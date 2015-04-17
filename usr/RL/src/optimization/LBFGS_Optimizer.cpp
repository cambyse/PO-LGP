#include "LBFGS_Optimizer.h"
#include "../util/lbfgs_codes.h"
#include "../util/util.h"
#include <config/Config.h>

#include <math.h> // for fabs()
#include <algorithm> // for min, max
#include <iomanip> // for std::setw(n)

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#define DEBUG_STRING "LBFGS: "
#include "../util/debug.h"

using std::min;
using std::max;

using util::Range;

LBFGS_Optimizer::LBFGS_Optimizer() {}

LBFGS_Optimizer::~LBFGS_Optimizer() {
    lbfgs_free(lbfgs_variables);
}

lbfgsfloatval_t LBFGS_Optimizer::static_objective(
    void *instance,
    const lbfgsfloatval_t *x,
    lbfgsfloatval_t *g,
    const int n,
    const lbfgsfloatval_t /*step*/
) {
    return ((LBFGS_Optimizer*)instance)->objective(x,g,n);
}

int LBFGS_Optimizer::static_progress(
    void *instance,
    const lbfgsfloatval_t *x,
    const lbfgsfloatval_t *g,
    const lbfgsfloatval_t fx,
    const lbfgsfloatval_t xnorm,
    const lbfgsfloatval_t gnorm,
    const lbfgsfloatval_t step,
    int n,
    int k,
    int ls
    ) {
    return ((LBFGS_Optimizer*)instance)->progress(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

lbfgsfloatval_t LBFGS_Optimizer::optimize(int * return_code, std::string * return_code_description) {

    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.orthantwise_c = l1_factor;
    param.delta = lbfgs_delta;
    param.epsilon = lbfgs_epsilon;
    if(l1_factor!=0) {
        param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
    }
    if(maximum_iterations>0) {
        param.max_iterations = maximum_iterations;
    }

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(number_of_variables, lbfgs_variables, &fx, static_objective, static_progress, this, &param);

    // write return code and description
    if(return_code!=nullptr) {
        *return_code=ret;
    }
    if(return_code_description!=nullptr) {
        *return_code_description=lbfgs_code(ret);
    }

    // Report the result.
    DEBUG_OUT(1,"L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");

    return fx;
}

void LBFGS_Optimizer::check_derivatives(
    const int& number_of_samples,
    const double& range,
    const double& delta,
    const double& allowed_maximum_relative_deviation,
    const bool use_current_values
    ) {

    // initialize arrays
    lbfgsfloatval_t * x = lbfgs_malloc(number_of_variables);
    lbfgsfloatval_t * grad = lbfgs_malloc(number_of_variables);
    lbfgsfloatval_t * grad_dummy = lbfgs_malloc(number_of_variables);

    // remember maximum relative deviation
    lbfgsfloatval_t achieved_maximum_relative_deviation = 0;

    // print inital info
    DEBUG_OUT(0,"===============================================================================================");
    DEBUG_OUT(0,"Checking first derivatives:");
    DEBUG_OUT(0,"    #samples                   " << number_of_samples );
    DEBUG_OUT(0,"    range                      " << range << (use_current_values?" (centered around current values)":" (centered around zero)") );
    DEBUG_OUT(0,"    delta                      " << delta );
    DEBUG_OUT(0,"    maximum allowed deviation  " << allowed_maximum_relative_deviation );
    DEBUG_OUT(1,"    -------------------------------------------------------------------------------------------");

    // draw and evaluate samples
    for(int sample_counter : Range(1,number_of_samples)) {

        // get variables
        for(int idx : Range(0,number_of_variables-1)) {
            x[idx] = range * (2*drand48()-1);
            if(use_current_values) {
                x[idx] += lbfgs_variables[idx];
            }
        }

        // calculate objective and analytical gradient at current point
        lbfgsfloatval_t fx = objective(x,grad,number_of_variables);
        DEBUG_OUT(1, "Sample " << sample_counter << " (fx = " << fx << ")" );
        DEBUG_OUT(1,"    -------------------------------------------------------------------------------------------");
            DEBUG_OUT(1,"    " <<
                      std::setw(5) << "idx" <<
                      std::setw(15) << "var" <<
                      std::setw(15) << "grad_a" <<
                      std::setw(15) << "grad_n" <<
                      std::setw(15) << "dev" <<
                      std::setw(15) << "dev/|grad|"
                );
        DEBUG_OUT(1,"    -------------------------------------------------------------------------------------------");

        // calculate numerical gradient
        for(int idx : Range(number_of_variables)) {

            // go in positive direction
            x[idx] += delta/2.;
            lbfgsfloatval_t fx_plus = objective(x,grad_dummy,number_of_variables);
            // go in negative direction
            x[idx] -= delta;
            lbfgsfloatval_t fx_minus = objective(x,grad_dummy,number_of_variables);
            // reset x
            x[idx] += delta/2.;

            // numerical gradient
            lbfgsfloatval_t ngrad = (fx_plus-fx_minus)/delta;

            // compute relative deviation and update maximum relative deviation
            lbfgsfloatval_t current_deviation = fabs(ngrad-grad[idx]);
            lbfgsfloatval_t current_relative_deviation = current_deviation/min(fabs(ngrad),fabs(grad[idx]));
            achieved_maximum_relative_deviation = max(achieved_maximum_relative_deviation,current_relative_deviation);

            // print result
            DEBUG_OUT(1,"    " <<
                      std::setw(5) << idx <<
                      std::setw(15) << x[idx] <<
                      std::setw(15) << grad[idx] <<
                      std::setw(15) << ngrad <<
                      std::setw(15) << current_deviation <<
                      std::setw(15) << current_relative_deviation
                );
        }
    }
    if(achieved_maximum_relative_deviation>allowed_maximum_relative_deviation) {
        DEBUG_OUT(0, "ERRORS in first derivative (" << achieved_maximum_relative_deviation << " > " << allowed_maximum_relative_deviation << ")" );
    } else {
        DEBUG_OUT(0, "No errors in first derivative (" << achieved_maximum_relative_deviation << " < " << allowed_maximum_relative_deviation << ")" );
    }
    DEBUG_OUT(0,"===============================================================================================");

    // free arrays
    lbfgs_free(x);
    lbfgs_free(grad);
    lbfgs_free(grad_dummy);
}

lbfgsfloatval_t LBFGS_Optimizer::objective(
    const lbfgsfloatval_t * /*x*/,
    lbfgsfloatval_t *g,
    const int n
) {
    DEBUG_ERROR("No objective function defined.");
    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }
    return fx;
}

int LBFGS_Optimizer::progress(
    const lbfgsfloatval_t *x,
    const lbfgsfloatval_t * /*g*/,
    const lbfgsfloatval_t fx,
    const lbfgsfloatval_t xnorm,
    const lbfgsfloatval_t /*gnorm*/,
    const lbfgsfloatval_t /*step*/,
    int n,
    int k,
    int /*ls*/
) {
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ")" );
    for(int x_idx=0; x_idx<n; ++x_idx) {
        DEBUG_OUT(1, "    x[" << x_idx << "] = " << x[x_idx]);
    }
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ")" );
    return 0;
}

LBFGS_Optimizer& LBFGS_Optimizer::set_number_of_variables(unsigned int n, bool zero) {
    if(number_of_variables!=n) {
        number_of_variables = n;
        lbfgs_free(lbfgs_variables);
        lbfgs_variables = lbfgs_malloc(n);
        if(zero) {
            for( int i : Range(n) ) {
                lbfgs_variables[i] = 0;
            }
        }
    }
    return *this;
}

LBFGS_Optimizer& LBFGS_Optimizer::set_maximum_iterations(unsigned int n) {
    maximum_iterations = n;
    return *this;
}

LBFGS_Optimizer& LBFGS_Optimizer::set_l1_factor(lbfgsfloatval_t f) {
    l1_factor = f;
    return *this;
}

LBFGS_Optimizer& LBFGS_Optimizer::set_lbfgs_delta(lbfgsfloatval_t f) {
    lbfgs_delta = f;
    return *this;
}

LBFGS_Optimizer& LBFGS_Optimizer::set_lbfgs_epsilon(lbfgsfloatval_t f) {
    lbfgs_epsilon = f;
    return *this;
}
