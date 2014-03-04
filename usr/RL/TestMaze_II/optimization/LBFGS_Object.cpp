#include "LBFGS_Object.h"

#include "../util/lbfgs_codes.h"
#include "../util/util.h"
#include "../Config.h"

#include <math.h> // for fabs()
#include <algorithm> // for min, max
#include <iomanip> // for std::setw(n)

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#define DEBUG_STRING "LBFGS: "
#include "../debug.h"

using std::min;
using std::max;

using util::Range;

LBFGS_Object::~LBFGS_Object() {
    lbfgs_free(variables);
}

lbfgsfloatval_t LBFGS_Object::optimize(objective_t obj,
                                       lbfgsfloatval_t * values,
                                       int * return_code,
                                       std::string * return_code_description
    ) {
    // Set objective
    if(obj!=nullptr) {
        objective = obj;
    }

    // Set inital values
    if(values!=nullptr) {
        for(int i : Range(number_of_variables)) {
            variables[i] = values[i];
        }
    }

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret;
    call_lbfgs(ret,fx);

    // write return code and description
    if(return_code!=nullptr) {
        *return_code=ret;
    }
    if(return_code_description!=nullptr) {
        *return_code_description=lbfgs_code(ret);
    }

    // Report the result.
    DEBUG_OUT(1,"L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");

    // Get values
    if(values!=nullptr) {
        for(int i : Range(number_of_variables)) {
            values[i] = variables[i];
        }
    }

    return fx;
}

lbfgsfloatval_t LBFGS_Object::optimize(std::vector<lbfgsfloatval_t> & values,
                                       int * return_code,
                                       std::string * return_code_description
    ) {

    // Set number of variables
    set_number_of_variables(values.size());

    // Set inital values
    for(int i : Range(number_of_variables)) {
        variables[i] = values[i];
    }

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret;
    call_lbfgs(ret,fx);

    // write return code and description
    if(return_code!=nullptr) {
        *return_code=ret;
    }
    if(return_code_description!=nullptr) {
        *return_code_description=lbfgs_code(ret);
    }

    // Report the result.
    DEBUG_OUT(1,"L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");

    // Get values
    for(int i : Range(number_of_variables)) {
        values[i] = variables[i];
    }

    return fx;
}

lbfgsfloatval_t LBFGS_Object::static_objective(
    void *instance,
    const lbfgsfloatval_t *x,
    lbfgsfloatval_t *g,
    const int n,
    const lbfgsfloatval_t /*step*/
    ) {
    if(((LBFGS_Object*)instance)->objective!=nullptr) {
        return ((LBFGS_Object*)instance)->objective(x,g);
    } else {
        DEBUG_WARNING("No objective function available.");
        for(int i : Range(n)) {
            g[i] = 0;
        }
        return 0;
    }
}

int LBFGS_Object::static_progress(
    void *instance,
    const lbfgsfloatval_t *x,
    const lbfgsfloatval_t *g,
    const lbfgsfloatval_t fx,
    const lbfgsfloatval_t xnorm,
    const lbfgsfloatval_t gnorm,
    const lbfgsfloatval_t step,
    int nr_variables,
    int iteration_nr,
    int ls
    ) {
    if(((LBFGS_Object*)instance)->progress!=nullptr) {
        return ((LBFGS_Object*)instance)->progress(x,g,fx,xnorm,gnorm,step,nr_variables,iteration_nr,ls);
    } else {
        DEBUG_OUT(1,"Iteration " << iteration_nr << " (fx = " << fx << ", xnorm = " << xnorm << ")" );
        for(int x_idx : Range(nr_variables)) {
            DEBUG_OUT(1, "    x[" << x_idx << "] = " << x[x_idx]);
            DEBUG_OUT(2, "    g[" << x_idx << "] = " << g[x_idx]);
        }
        DEBUG_OUT(1,"Iteration " << iteration_nr << " (fx = " << fx << ", xnorm = " << xnorm << ")" );
        return 0;
    }
}

void LBFGS_Object::check_derivatives(
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
                x[idx] += variables[idx];
            }
        }

        // calculate objective and analytical gradient at current point
        lbfgsfloatval_t fx = objective(x,grad);
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
            lbfgsfloatval_t fx_plus = objective(x,grad_dummy);
            // go in negative direction
            x[idx] -= delta;
            lbfgsfloatval_t fx_minus = objective(x,grad_dummy);
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

LBFGS_Object& LBFGS_Object::set_objective(objective_t obj) {
    objective = obj;
    return *this;
}

LBFGS_Object& LBFGS_Object::set_progress(progress_t pro) {
    progress = pro;
    return *this;
}

LBFGS_Object& LBFGS_Object::set_number_of_variables(unsigned int n, bool zero) {
    if(number_of_variables!=n) {
        number_of_variables = n;
        lbfgs_free(variables);
        variables = lbfgs_malloc(n);
        if(zero) {
            for(int i : Range(n)) {
                variables[i] = 0;
            }
        }
    }
    return *this;
}

LBFGS_Object& LBFGS_Object::set_maximum_iterations(unsigned int n) {
    maximum_iterations = n;
    return *this;
}

LBFGS_Object& LBFGS_Object::set_l1_factor(lbfgsfloatval_t f) {
    l1_factor = f;
    return *this;
}

LBFGS_Object& LBFGS_Object::set_delta(lbfgsfloatval_t f) {
    delta = f;
    return *this;
}

LBFGS_Object& LBFGS_Object::set_epsilon(lbfgsfloatval_t f) {
    epsilon = f;
    return *this;
}

void LBFGS_Object::call_lbfgs(int & ret, lbfgsfloatval_t & fx) {
    // Initialize the parameters
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.orthantwise_c = l1_factor;
    param.delta = delta;
    param.epsilon = epsilon;
    if(l1_factor!=0) {
        param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
    }
    if(maximum_iterations>0) {
        param.max_iterations = maximum_iterations;
    }

    // Start the L-BFGS optimization
    ret = lbfgs(number_of_variables, variables, &fx, static_objective, static_progress, this, &param);
}
