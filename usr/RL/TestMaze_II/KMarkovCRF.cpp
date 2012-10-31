/*
 * KMarkovCRF.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: robert
 */

#include "KMarkovCRF.h"

#define DEBUG_STRING ""
#define DEBUG_LEVEL 1
#include "debug.h"

KMarkovCRF::KMarkovCRF(
        const int& kk,
        const int& dim_x,
        const int& dim_y,
        const int& action_n): k(kk) {

    lambda_size = dim_x*dim_y*action_n*dim_x*dim_y;
    lambda = lbfgs_malloc(lambda_size);
    if (lambda == NULL) {
        printf("ERROR: Failed to allocate a memory block for variables.\n");
        return;
    }

    for(int state_from_idx=0; state_from_idx<dim_x*dim_y; ++state_from_idx) {
        for(int action_idx=0; action_idx<action_n; ++action_idx) {
            for(int state_to_idx=0; state_to_idx<dim_x*dim_y; ++state_to_idx) {
                int linear_idx = state_to_idx + dim_x*dim_y*action_idx + dim_x*dim_y*action_n*state_from_idx;
                lambda[linear_idx] = 1;
                state_features.push_back(MDPFeature(state_from_idx,action_idx,state_to_idx));
                state_parameter_indices.push_back(linear_idx);
            }
        }
    }
}

KMarkovCRF::~KMarkovCRF() {
    lbfgs_free(lambda);
}

lbfgsfloatval_t KMarkovCRF::static_evaluate(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
) {
    return ((KMarkovCRF*)instance)->evaluate(x,g,n);
}

lbfgsfloatval_t KMarkovCRF::evaluate(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {
    int i;
    lbfgsfloatval_t fx = 0.0;

    for (i = 0;i < n;i += 2) {
//        lbfgsfloatval_t t1 = 1.0 - x[i];
//        lbfgsfloatval_t t2 = 10.0 * (x[i+1] - x[i] * x[i]);
//        g[i+1] = 20.0 * t2;
//        g[i] = -2.0 * (x[i] * g[i+1] + t1);
//        fx += t1 * t1 + t2 * t2;
        fx += x[i]*x[i] + x[i+1]*x[i+1]*x[i+1];
        g[i] = 2*x[i];
        g[i+1] = 3*x[i+1]*x[i+1];
    }
    return fx;
}

int KMarkovCRF::static_progress(
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
    return ((KMarkovCRF*)instance)->progress(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

int KMarkovCRF::progress(
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

    printf("Iteration %d:\n", k);
    printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
    printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
    printf("\n");
    return 0;

//    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << "):");
//    for(int idx=0; idx<n; ++idx) {
//        DEBUG_OUT(1,"    x[" << idx << "] = " << x[idx] << ", grad[" << idx << "] = " << g[idx]);
//    }
//    DEBUG_OUT(1,"");
}

int KMarkovCRF::optimize() {

    int i, ret = 0;
    lbfgsfloatval_t fx;
//    lbfgsfloatval_t *x = lbfgs_malloc(lambda_size);
    lbfgs_parameter_t param;

//    if (x == NULL) {
//        printf("ERROR: Failed to allocate a memory block for variables.\n");
//        return 1;
//    }

    /* Initialize the variables. */
    for (i = 0;i < lambda_size;i += 2) {
        lambda[i] = -1.2;
        lambda[i+1] = 1.0;
    }

    /* Initialize the parameters for the L-BFGS optimization. */
    lbfgs_parameter_init(&param);
    /*param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;*/

    /*
           Start the L-BFGS optimization; this will invoke the callback functions
           evaluate() and progress() when necessary.
     */
    ret = lbfgs(lambda_size, lambda, &fx, static_evaluate, static_progress, this, &param);

    /* Report the result. */
    printf("L-BFGS optimization terminated with status code = %d\n", ret);
    printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, lambda[0], lambda[1]);

//    lbfgs_free(x);
    return 0;
}

void KMarkovCRF::add_action_state_reward_tripel(
        const int& action,
        const int& state,
        const double& reward
) {
    episode_data.push_back(std::make_tuple(action,state,reward));
    DEBUG_OUT(1, "added (action,state,reward) = (" << action << "," << state << "," << reward << ")" );
}

void KMarkovCRF::check_derivative(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation) {
//     save current parameters
//    lbfgsfloatval_t * x_tmp = lbfgs_malloc(lambda_size);
//    for(int x_idx=0; x_idx<lambda_size; ++x_idx) {
//        x_tmp[x_idx] = x[idx];
//    }

    // initialize arrays
    lbfgsfloatval_t * x = lbfgs_malloc(lambda_size);
    lbfgsfloatval_t * dx = lbfgs_malloc(lambda_size);
    lbfgsfloatval_t * grad = lbfgs_malloc(lambda_size);
    lbfgsfloatval_t * grad_dummy = lbfgs_malloc(lambda_size);

    // remember deviations
    lbfgsfloatval_t relative_deviation = 0;

    DEBUG_OUT(0,"Checking first derivative (#samples="<<number_of_samples<<", range=+/-"<<range<<", max_var="<<max_variation<<", max_rel_dev="<<max_relative_deviation);
    for(int count=0; count<number_of_samples; ++count) {

        // set test point
        for(int x_idx=0; x_idx<lambda_size; ++x_idx) {
            x[x_idx] = range * (2*drand48()-1);
            dx[x_idx] = (2*drand48()-1)*max_variation;
        }

        lbfgsfloatval_t fx = evaluate(x,grad,lambda_size);
        DEBUG_OUT(1, "fx = " << fx );
        for(int x_idx=0; x_idx<lambda_size; ++x_idx) {

            // go in positive direction
            x[x_idx] += dx[x_idx]/2.;
            lbfgsfloatval_t fx_plus = evaluate(x,grad_dummy,lambda_size);
            // go in negative direction
            x[x_idx] -= dx[x_idx];
            lbfgsfloatval_t fx_minus = evaluate(x,grad_dummy,lambda_size);
            // reset x
            x[x_idx] += dx[x_idx]/2.;

            // print result
            lbfgsfloatval_t ngrad = (fx_plus-fx_minus)/dx[x_idx];
            DEBUG_OUT(1,
                    "    diff[" << x_idx << "] = " << grad[x_idx]-ngrad <<
                    ", grad["   << x_idx << "] = " << grad[x_idx] <<
                    ", ngrad["  << x_idx << "] = " << ngrad <<
                    ", x["      << x_idx << "] = " << x[x_idx] <<
                    ", dx["     << x_idx << "] = " << dx[x_idx]
            );

            // check for deviations
            if(fabs(ngrad-grad[x_idx])/fabs(grad[x_idx])>relative_deviation) {
                relative_deviation=fabs(ngrad-grad[x_idx])/fabs(grad[x_idx]);
            }
        }
    }
    if(relative_deviation>max_relative_deviation) {
        DEBUG_OUT(0, "ERRORS in first derivative found: max relative deviation = " << relative_deviation << "(tolerance = " << max_relative_deviation << ")" );
        DEBUG_OUT(0, "");
    } else {
        DEBUG_OUT(0, "No error in first derivative found (no relative deviations larger that " << max_relative_deviation << ").");
        DEBUG_OUT(0, "");
    }
    lbfgs_free(x);
    lbfgs_free(dx);
    lbfgs_free(grad);
    lbfgs_free(grad_dummy);
}

double KMarkovCRF::raw_state_probability(const int& state_from, const int& action, const int& state_to) {
    double prod = 1;
    for(uint f_idx=0; f_idx<state_features.size(); ++f_idx) {
        prod *= state_features[f_idx].evaluate(std::make_tuple(state_from,action),state_to);
    }
    return prod;
}

