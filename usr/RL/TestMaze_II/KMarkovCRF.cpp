/*
 * KMarkovCRF.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: robert
 */

#include "KMarkovCRF.h"

#define DEBUG_STRING ""
#define DEBUG_LEVEL 0
#include "debug.h"

KMarkovCRF::KMarkovCRF(
        const int& kk,
        const int& x,
        const int& y,
        const int& a_n):
        k(kk), x_dim(x), y_dim(y), action_n(a_n) {

    lambda_size = x_dim*y_dim*action_n*x_dim*y_dim;
    lambda = lbfgs_malloc(lambda_size);
    if (lambda == nullptr) {
        printf("ERROR: Failed to allocate a memory block for variables.\n");
        return;
    }

    for(int state_from_idx=0; state_from_idx<x_dim*y_dim; ++state_from_idx) {
        for(int action_idx=0; action_idx<action_n; ++action_idx) {
            for(int state_to_idx=0; state_to_idx<x_dim*y_dim; ++state_to_idx) {
                int linear_idx = state_to_idx + x_dim*y_dim*action_idx + x_dim*y_dim*action_n*state_from_idx;
                lambda[linear_idx] = 0;
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
    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }

    for(uint d=k; d<episode_data.size(); ++d) {
        int state_from = std::get<1>(episode_data[d-1]);
        int action = std::get<0>(episode_data[d]);
        int state_to = std::get<1>(episode_data[d]);
        fx += log(state_probability(state_from,action,state_to,x));
        for(uint f_idx=0; f_idx<state_features.size(); ++f_idx) {
            g[state_parameter_indices[f_idx]] += state_features[f_idx].evaluate(std::make_tuple(state_from,action),state_to);
            for(int state_other=0; state_other<x_dim*y_dim; ++state_other) {
                g[state_parameter_indices[f_idx]] -= state_probability(state_from,action,state_other,x)*state_features[f_idx].evaluate(std::make_tuple(state_from,action),state_other);
            }
        }
    }

    // blfgs minimizes but we want to maximize
    fx *= -1;
    for(int i=0; i<n; i++) {
        g[i] *= -1;
    }

    return fx;

    /* old evaluate function */
//    int i;
//    lbfgsfloatval_t fx = 0.0;
//    for (i = 0;i < n;i += 2) {
//        fx += x[i]*x[i] + x[i+1]*x[i+1]*x[i+1];
//        g[i] = 2*x[i];
//        g[i+1] = 3*x[i+1]*x[i+1];
//    }
//    return fx;
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

    DEBUG_OUT(0,"Iteration " << k << " (fx = " << fx << "):");
    for(uint f_idx=0; f_idx<state_features.size(); ++f_idx) {
        MDPFeature f = state_features[f_idx];
        int l_idx = state_parameter_indices[f_idx];
        DEBUG_OUT(1, "    f(" <<
                f.get_state_from() << "," <<
                f.get_action() << "," <<
                f.get_state_to() << ") --> t[" <<
                l_idx << "] = " <<
                x[l_idx]);
    }
    DEBUG_OUT(1,"");

    return 0;

    /* old progress function */
//    printf("Iteration %d:\n", k);
//    printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
//    printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
//    printf("\n");
//    return 0;
}

void KMarkovCRF::optimize() {
    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    // param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(lambda_size, lambda, &fx, static_evaluate, static_progress, this, &param);

    // Report the result.
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx/(episode_data.size()-k)) );
    for(uint f_idx=0; f_idx<state_features.size(); ++f_idx) {
        MDPFeature f = state_features[f_idx];
        int l_idx = state_parameter_indices[f_idx];
        DEBUG_OUT(1, "    f(" <<
                f.get_state_from() << "," <<
                f.get_action() << "," <<
                f.get_state_to() << ") --> t[" <<
                l_idx << "] = " <<
                lambda[l_idx]);
    }
    DEBUG_OUT(0,"");
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

double KMarkovCRF::state_probability(const int& state_from, const int& action, const int& state_to, lbfgsfloatval_t const * x) {
    return raw_state_probability(state_from,action,state_to,x)/partition_function(state_from,action,x);
}

double KMarkovCRF::raw_state_probability(const int& state_from, const int& action, const int& state_to, lbfgsfloatval_t const * x) {
    if(x==nullptr) x = lambda;
    double prod = 1;
    for(uint f_idx=0; f_idx<state_features.size(); ++f_idx) {
        prod *= exp( x[state_parameter_indices[f_idx]] * state_features[f_idx].evaluate(std::make_tuple(state_from,action),state_to) );
    }
    return prod;
}

double KMarkovCRF::partition_function(const int& state_from, const int& action, lbfgsfloatval_t const * x) {
    if(x==nullptr) x = lambda;
    double z = 0;
    for(int state_to=0; state_to<x_dim*y_dim; ++ state_to) {
        z += raw_state_probability(state_from,action,state_to,x);
    }
    return z;
}


