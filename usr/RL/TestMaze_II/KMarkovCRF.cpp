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
        const int& action_n): k(kk), z_up_to_date(false) {

    // only one factor
    Factor<std::tuple<int,int>,int > mdpFactor;

    // initialize parameters
    lambda_size = dim_x*dim_y*action_n*dim_x*dim_y;
    lambda = lbfgs_malloc(lambda_size);
    if (lambda == NULL) {
        DEBUG_OUT(0,"ERROR: Failed to allocate a memory block for variables.");
        return;
    }
    mdpFactor.lambda = lambda;

    for(int state_from_idx=0; state_from_idx<dim_x*dim_y; ++state_from_idx) {
        for(int action_idx=0; action_idx<action_n; ++action_idx) {
            for(int state_to_idx=0; state_to_idx<dim_x*dim_y; ++state_to_idx) {
                int linear_idx = state_to_idx + dim_x*dim_y*action_idx + dim_x*dim_y*action_n*state_from_idx;
                lambda[linear_idx] = 1;
                mdpFactor.features.push_back(SimpleMDPFeature(state_from_idx,action_idx,state_to_idx));
            }
        }
    }
    state_factors.push_back(mdpFactor);
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


//    lbfgsfloatval_t fx = 0;
//    for(int idx=0; idx<n; ++idx) {
//        fx += x[idx]*x[idx]*x[idx];
//        g[idx] = 3*x[idx]*x[idx];
//    }
//    return fx;

    int i;
    lbfgsfloatval_t fx = 0.0;

    for (i = 0;i < n;i += 2) {
        lbfgsfloatval_t t1 = 1.0 - x[i];
        lbfgsfloatval_t t2 = 10.0 * (x[i+1] - x[i] * x[i]);
        g[i+1] = 20.0 * t2;
        g[i] = -2.0 * (x[i] * g[i+1] + t1);
        fx += t1 * t1 + t2 * t2;
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
    DEBUG_OUT(1,"Iteration " << k << ":");
    for(int idx=0; idx<n; ++idx) {
        DEBUG_OUT(1,"    x[" << idx << "] = " << x[idx] << ", grad[" << idx << "] = " << g[idx]);
    }
    DEBUG_OUT(1,"");

        /* DEBUG_OUT(1,"Iteration " << k << ":");
    DEBUG_OUT(1,"  fx = " << fx << ", x[0] = " << x[0] << ", x[1] = " << x[1] );
    DEBUG_OUT(1,"  xnorm = " << xnorm << ", gnorm = " << gnorm << ", step = " << step );
    DEBUG_OUT(1,"");
    return 0; */
}

int KMarkovCRF::optimize() {
    int i, ret = 0;
    lbfgsfloatval_t fx;
    lbfgsfloatval_t *x = lbfgs_malloc(lambda_size);
    lbfgs_parameter_t param;

    for(int i=0; i<lambda_size; ++i) {
        x[i] = 2;
    }

//    if (x == NULL) {
//        DEBUG_OUT(0,"ERROR: Failed to allocate a memory block for variables.");
//        return 1;
//    }

    /* Initialize the variables. */
//    for (i = 0;i < N;i += 2) {
//        x[i] = -1.2;
//        x[i+1] = 1.0;
//    }

    /* Initialize the parameters for the L-BFGS optimization. */
    lbfgs_parameter_init(&param);
    /*param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;*/

    /*
        Start the L-BFGS optimization; this will invoke the callback functions
        evaluate() and progress() when necessary.
     */
    ret = lbfgs(lambda_size, x, &fx, static_evaluate, static_progress, this, NULL);
//    ret = lbfgs(N, x, &fx, static_evaluate, static_progress, this, &param);

    /* Report the result. */
    DEBUG_OUT(1,"L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(1,"    fx = " << fx );
    for(int idx=0; idx<lambda_size; ++idx) {
        DEBUG_OUT(1,"    lambda[" << idx << "] = " << x[idx]);
    }

    lbfgs_free(x);
    return 0;
}

void KMarkovCRF::add_action_state_reward_tripel(
        const int& state,
        const int& action,
        const double& reward
) {
    episode_data.push_back(std::make_tuple(state,action,reward));
}

void KMarkovCRF::check_derivative(const int& number_of_samples, const double& range, const double& max_difference) {
//     save current parameters
//    lbfgsfloatval_t * lambda_tmp = lbfgs_malloc(lambda_size);
//    for(int lambda_idx=0; lambda_idx<lambda_size; ++lambda_idx) {
//        lambda_tmp[lambda_idx] = lambda[idx];
//    }

    // test different sets of lambda
    lbfgsfloatval_t * lambda_test = lbfgs_malloc(lambda_size);
    lbfgsfloatval_t * dlambda_vector = lbfgs_malloc(lambda_size);
    lbfgsfloatval_t * grad = lbfgs_malloc(lambda_size);
    bool significant_deviation_found = false;
    DEBUG_OUT(0,"Checking first derivative by finite differences:");
    for(int count=0; count<number_of_samples; ++count) {
        for(int lambda_idx=0; lambda_idx<lambda_size; ++lambda_idx) {
            lambda_test[lambda_idx] = range * (2*drand48()-1);
            dlambda_vector[lambda_idx] = (2*drand48()-1)*max_difference;
        }
        lbfgsfloatval_t fx_1 = evaluate(lambda_test,grad,lambda_size);
        std::cout << "fx_1 = " << fx_1 << std::endl;
        for(int lambda_idx=0; lambda_idx<lambda_size; ++lambda_idx) {
            lambda_test[lambda_idx] += dlambda_vector[lambda_idx];
            lbfgsfloatval_t fx_2 = evaluate(lambda_test,grad,lambda_size);
            lambda_test[lambda_idx] -= dlambda_vector[lambda_idx];
            lbfgsfloatval_t dfx = fx_2-fx_1;
            std::cout <<
                    "    diff["      << lambda_idx << "] = " << grad[lambda_idx]-dfx/dlambda_vector[lambda_idx] <<
                    ", grad["        << lambda_idx << "] = " << grad[lambda_idx] <<
                    ", dfx/dlambda[" << lambda_idx << "] = " << dfx/dlambda_vector[lambda_idx] <<
                    ", lambda["      << lambda_idx << "] = " << lambda_test[lambda_idx] <<
                    ", dlambda["     << lambda_idx << "] = " << dlambda_vector[lambda_idx] <<
                    ", dfx["         << lambda_idx << "] = " << dfx <<
                    std::endl;
            if(fabs(grad[lambda_idx]-dfx/dlambda_vector[lambda_idx])>max_difference) {
                significant_deviation_found = true;
            }
        }
    }
    if(significant_deviation_found) {
        std::cout << "ERRORS in first derivative found (deviations larger that " << max_difference << ")." << std::endl << std::endl;
    } else {
        std::cout << "No error in first derivative found (no deviations larger that " << max_difference << ")." << std::endl << std::endl;
    }
    lbfgs_free(lambda_test);
    lbfgs_free(dlambda_vector);
    lbfgs_free(grad);
}

double KMarkovCRF::update_z() {
    z = 1;
    return z;
}

double KMarkovCRF::raw_state_probability(const int& state_from, const int& action, const int& state_to) {
    double prod = 1;
    for(uint f_idx=0; f_idx<state_factors.size(); ++f_idx) {
        prod *= state_factors[f_idx].evaluate(std::make_tuple(state_from,action),state_to);
    }
    return prod;
}

