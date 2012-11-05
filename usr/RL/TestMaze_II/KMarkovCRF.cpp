///*
/* KMarkovCRF.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: robert
 */

#include "KMarkovCRF.h"

#define DEBUG_STRING ""
#define DEBUG_LEVEL 1
#include "debug.h"

const char* KMarkovCRF::action_strings[5] = { "STAY ", "UP   ", "DOWN ", "LEFT ", "RIGHT" };

KMarkovCRF::KMarkovCRF(
        const int& k_state_,
        const int& k_reward_,
        const int& k_,
        const int& x,
        const int& y,
        const int& a_n):
        k_state(k_state_), k_reward(k_reward_), k(k_), x_dim(x), y_dim(y), action_n(a_n) {

    lambda_state_size = x_dim*y_dim*action_n*x_dim*y_dim;
    lambda_reward_size = x_dim*y_dim*k_reward;
    lambda_size = (k+1)*(x_dim*y_dim + action_n + 2);
    lambda_state = lbfgs_malloc(lambda_state_size);
    lambda_reward = lbfgs_malloc(lambda_reward_size);
    lambda = lbfgs_malloc(lambda_size);
    if (lambda_state == nullptr || lambda_reward == nullptr || lambda == nullptr) {
        printf("ERROR: Failed to allocate a memory block for variables.\n");
        lbfgs_free(lambda_state);
        lbfgs_free(lambda_reward);
        lbfgs_free(lambda);
        return;
    }

    int linear_state_idx = 0;
    int linear_reward_idx = 0;
    for(int state_from_idx=0; state_from_idx<x_dim*y_dim; ++state_from_idx) {
        // rewards
        for(int k_idx=0; k_idx<k_reward; ++k_idx) {
            lambda_reward[linear_reward_idx] = 0;
            reward_features.push_back(OldRewardFeature(state_from_idx,k_idx));
            reward_parameter_indices.push_back(linear_reward_idx);
            ++linear_reward_idx;
        }
        // states
        for(int action_idx=0; action_idx<action_n; ++action_idx) {
            for(int state_to_idx=0; state_to_idx<x_dim*y_dim; ++state_to_idx) {
                lambda_state[linear_state_idx] = 0;
                state_features.push_back(MDPFeature(state_from_idx,action_idx,state_to_idx));
                state_parameter_indices.push_back(linear_state_idx);
                ++linear_state_idx;
            }
        }
    }

    int linear_idx = 0;
    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx<=k; ++k_idx) {
        // actions
        for(action_t action=0; action<action_n; ++action) {
            lambda[linear_idx] = 0;
            features.push_back(ActionFeature(action,k_idx));
            parameter_indices.push_back(linear_idx);
            ++linear_idx;
        }
        // states
        for(state_t state=0; state<x_dim*y_dim; ++state) {
            lambda[linear_idx] = 0;
            features.push_back(StateFeature(state,k_idx));
            parameter_indices.push_back(linear_idx);
            ++linear_idx;
        }
        // reward
        for(reward_t reward = 0.0; reward<=1.0; ++reward) {
            lambda[linear_idx] = 0;
            features.push_back(RewardFeature(reward,k_idx));
            parameter_indices.push_back(linear_idx);
            ++linear_idx;
        }
    }
}

KMarkovCRF::~KMarkovCRF() {
    lbfgs_free(lambda_state);
    lbfgs_free(lambda_reward);
    lbfgs_free(lambda);
}

lbfgsfloatval_t KMarkovCRF::static_evaluate_state_model(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t /*step*/
) {
    return ((KMarkovCRF*)instance)->evaluate_state_model(x,g,n);
}

lbfgsfloatval_t KMarkovCRF::static_evaluate_reward_model(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t /*step*/
) {
    return ((KMarkovCRF*)instance)->evaluate_reward_model(x,g,n);
}

lbfgsfloatval_t KMarkovCRF::static_evaluate_model(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t /*step*/
) {
    return ((KMarkovCRF*)instance)->evaluate_model(x,g,n);
}

lbfgsfloatval_t KMarkovCRF::evaluate_state_model(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {
    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }
    for(uint d=k_state-1; d<episode_data.size(); ++d) {
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
}

lbfgsfloatval_t KMarkovCRF::evaluate_reward_model(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {
    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }
    for(uint d=k_reward-1; d<episode_data.size(); ++d) {
        std::vector<int> state_vector;
        for(int delay=0; delay<k_reward; ++delay) {
            state_vector.push_back(std::get<1>(episode_data[d-delay]));
        }
        double reward = std::get<2>(episode_data[d]);
        fx += log(reward_probability(state_vector,reward,x));
        for(uint f_idx=0; f_idx<reward_features.size(); ++f_idx) {
            g[reward_parameter_indices[f_idx]] += reward_features[f_idx].evaluate(state_vector,reward);
            for(double reward_other=0; reward_other<=1; reward_other+=1) {
                g[reward_parameter_indices[f_idx]] -= reward_probability(state_vector,reward_other,x)*reward_features[f_idx].evaluate(state_vector,reward_other);
            }
        }
    }
    // blfgs minimizes but we want to maximize
    fx *= -1;
    for(int i=0; i<n; i++) {
        g[i] *= -1;
    }

    return fx;
}

lbfgsfloatval_t KMarkovCRF::evaluate_model(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {
    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }
    for(uint data_idx=k; data_idx<episode_data.size(); ++data_idx) {
        episode_t episode;
        for(int delay=-k; delay<=0; ++delay) {
            episode.push_back(episode_data[data_idx+delay]);
        }
        fx += log(probability(episode,x));
        for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
            g[parameter_indices[f_idx]] += features[f_idx].evaluate(episode);
            episode_t episode_copy = episode;
            for(reward_t reward_predict=0.0; reward_predict<=1.0; ++reward_predict) {
                for(int state_predict=0; state_predict<x_dim*y_dim; ++state_predict) {
                    std::get<2>(episode_copy.back()) = reward_predict;
                    std::get<1>(episode_copy.back()) = state_predict;
                    g[parameter_indices[f_idx]] -= probability(episode_copy,x)*features[f_idx].evaluate(episode_copy);
                }
            }
        }
    }
    // use NEGATIVE log likelihood (blfgs minimizes the objective)
    fx *= -1;
    for(int i=0; i<n; i++) {
        g[i] *= -1;
    }

    return fx;
}

int KMarkovCRF::static_progress_state_model(
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
    return ((KMarkovCRF*)instance)->progress_state_model(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

int KMarkovCRF::static_progress_reward_model(
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
    return ((KMarkovCRF*)instance)->progress_reward_model(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

int KMarkovCRF::static_progress_model(
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
    return ((KMarkovCRF*)instance)->progress_model(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

int KMarkovCRF::progress_state_model(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t * /*g*/,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t /*xnorm*/,
        const lbfgsfloatval_t /*gnorm*/,
        const lbfgsfloatval_t /*step*/,
        int /*n*/,
        int k,
        int /*ls*/
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
}

int KMarkovCRF::progress_reward_model(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t * /*g*/,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t /*xnorm*/,
        const lbfgsfloatval_t /*gnorm*/,
        const lbfgsfloatval_t /*step*/,
        int /*n*/,
        int k,
        int /*ls*/
) {
    DEBUG_OUT(0,"Iteration " << k << " (fx = " << fx << "):");
    for(uint f_idx=0; f_idx<reward_features.size(); ++f_idx) {
        OldRewardFeature f = reward_features[f_idx];
        int l_idx = reward_parameter_indices[f_idx];
        DEBUG_OUT(1, "    f(" <<
                f.get_state() << "," <<
                f.get_delay() << ") --> t[" <<
                l_idx << "] = " <<
                x[l_idx]);
    }
    DEBUG_OUT(1,"");

    return 0;
}

int KMarkovCRF::progress_model(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t * /*g*/,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t /*xnorm*/,
        const lbfgsfloatval_t /*gnorm*/,
        const lbfgsfloatval_t /*step*/,
        int /*n*/,
        int k,
        int /*ls*/
) {
    DEBUG_OUT(0,"Iteration " << k << " (fx = " << fx << "):");
    for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
        Feature<episode_t> f = features[f_idx];
        int l_idx = parameter_indices[f_idx];
        DEBUG_OUT(1, "    f" <<
                f.identifier() <<
                " --> t[" <<
                l_idx << "] = " <<
                x[l_idx]);
    }
    DEBUG_OUT(1,"");

    return 0;
}

void KMarkovCRF::optimize_state_model() {
    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    // param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(lambda_state_size, lambda_state, &fx, static_evaluate_state_model, static_progress_state_model, this, &param);

    // Report the result.
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx/(episode_data.size()-k_state)) );
    for(uint f_idx=0; f_idx<state_features.size(); ++f_idx) {
        MDPFeature f = state_features[f_idx];
        int l_idx = state_parameter_indices[f_idx];
        DEBUG_OUT(1, "    f(" <<
                f.get_state_from() << "," <<
                f.get_action() << "," <<
                f.get_state_to() << ") --> t[" <<
                l_idx << "] = " <<
                lambda_state[l_idx]);
    }
    DEBUG_OUT(0,"");
}

void KMarkovCRF::optimize_reward_model() {
    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    // param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(lambda_reward_size, lambda_reward, &fx, static_evaluate_reward_model, static_progress_reward_model, this, &param);

    // Report the result.
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx/(episode_data.size()-k_reward)) );
    for(uint f_idx=0; f_idx<reward_features.size(); ++f_idx) {
        OldRewardFeature f = reward_features[f_idx];
        int l_idx = reward_parameter_indices[f_idx];
        DEBUG_OUT(1, "    f(" <<
                f.get_state() << "," <<
                f.get_delay() << ") --> t[" <<
                l_idx << "] = " <<
                lambda_reward[l_idx]);
    }
    DEBUG_OUT(0,"");
}

void KMarkovCRF::optimize_model() {
    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    // param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(lambda_size, lambda, &fx, static_evaluate_model, static_progress_model, this, &param);

    // Report the result.
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx/(episode_data.size()-k)) );
    for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
        Feature<episode_t> f = features[f_idx];
        int l_idx = parameter_indices[f_idx];
        DEBUG_OUT(1, "    f" <<
                f.identifier() <<
                " --> t[" <<
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

void KMarkovCRF::check_state_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation) {
    // initialize arrays
    lbfgsfloatval_t * x = lbfgs_malloc(lambda_state_size);
    lbfgsfloatval_t * dx = lbfgs_malloc(lambda_state_size);
    lbfgsfloatval_t * grad = lbfgs_malloc(lambda_state_size);
    lbfgsfloatval_t * grad_dummy = lbfgs_malloc(lambda_state_size);

    // remember deviations
    lbfgsfloatval_t relative_deviation = 0;

    DEBUG_OUT(0,"Checking first derivative (#samples="<<number_of_samples<<", range=+/-"<<range<<", max_var="<<max_variation<<", max_rel_dev="<<max_relative_deviation);
    for(int count=0; count<number_of_samples; ++count) {

        // set test point
        for(int x_idx=0; x_idx<lambda_state_size; ++x_idx) {
            x[x_idx] = range * (2*drand48()-1);
            dx[x_idx] = (2*drand48()-1)*max_variation;
        }

        lbfgsfloatval_t fx = evaluate_state_model(x,grad,lambda_state_size);
        DEBUG_OUT(1, "fx = " << fx );
        for(int x_idx=0; x_idx<lambda_state_size; ++x_idx) {

            // go in positive direction
            x[x_idx] += dx[x_idx]/2.;
            lbfgsfloatval_t fx_plus = evaluate_state_model(x,grad_dummy,lambda_state_size);
            // go in negative direction
            x[x_idx] -= dx[x_idx];
            lbfgsfloatval_t fx_minus = evaluate_state_model(x,grad_dummy,lambda_state_size);
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

void KMarkovCRF::check_reward_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation) {
    // initialize arrays
    lbfgsfloatval_t * x = lbfgs_malloc(lambda_reward_size);
    lbfgsfloatval_t * dx = lbfgs_malloc(lambda_reward_size);
    lbfgsfloatval_t * grad = lbfgs_malloc(lambda_reward_size);
    lbfgsfloatval_t * grad_dummy = lbfgs_malloc(lambda_reward_size);

    // remember deviations
    lbfgsfloatval_t relative_deviation = 0;

    DEBUG_OUT(0,"Checking first derivative (#samples="<<number_of_samples<<", range=+/-"<<range<<", max_var="<<max_variation<<", max_rel_dev="<<max_relative_deviation);
    for(int count=0; count<number_of_samples; ++count) {

        // set test point
        for(int x_idx=0; x_idx<lambda_reward_size; ++x_idx) {
            x[x_idx] = range * (2*drand48()-1);
            dx[x_idx] = (2*drand48()-1)*max_variation;
        }

        lbfgsfloatval_t fx = evaluate_reward_model(x,grad,lambda_reward_size);
        DEBUG_OUT(1, "fx = " << fx );
        for(int x_idx=0; x_idx<lambda_reward_size; ++x_idx) {

            // go in positive direction
            x[x_idx] += dx[x_idx]/2.;
            lbfgsfloatval_t fx_plus = evaluate_reward_model(x,grad_dummy,lambda_reward_size);
            // go in negative direction
            x[x_idx] -= dx[x_idx];
            lbfgsfloatval_t fx_minus = evaluate_reward_model(x,grad_dummy,lambda_reward_size);
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

void KMarkovCRF::check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation) {
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

        lbfgsfloatval_t fx = evaluate_model(x,grad,lambda_size);
        DEBUG_OUT(1, "fx = " << fx );
        for(int x_idx=0; x_idx<lambda_size; ++x_idx) {

            // go in positive direction
            x[x_idx] += dx[x_idx]/2.;
            lbfgsfloatval_t fx_plus = evaluate_model(x,grad_dummy,lambda_size);
            // go in negative direction
            x[x_idx] -= dx[x_idx];
            lbfgsfloatval_t fx_minus = evaluate_model(x,grad_dummy,lambda_size);
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
    return raw_state_probability(state_from,action,state_to,x)/state_partition_function(state_from,action,x);
}

double KMarkovCRF::reward_probability(const std::vector<int> & state_vector, const double& rew, lbfgsfloatval_t const * x) {
    return raw_reward_probability(state_vector,rew,x)/reward_partition_function(state_vector,x);
}

KMarkovCRF::probability_t KMarkovCRF::probability(const episode_t& episode, lbfgsfloatval_t const * x) {
    if(episode.size()==0) {
        DEBUG_OUT(0,"Cannot calculate probability given lenth-zero episode.");
        return 0;
    }
    return raw_probability(episode,x)/partition_function(episode,x);
}

double KMarkovCRF::raw_state_probability(const int& state_from, const int& action, const int& state_to, lbfgsfloatval_t const * x) {
    if(x==nullptr) x = lambda_state;
    double prod = 1;
    for(uint f_idx=0; f_idx<state_features.size(); ++f_idx) {
        prod *= exp( x[state_parameter_indices[f_idx]] * state_features[f_idx].evaluate(std::make_tuple(state_from,action),state_to) );
    }
    return prod;
}

double KMarkovCRF::state_partition_function(const int& state_from, const int& action, lbfgsfloatval_t const * x) {
    if(x==nullptr) x = lambda_state;
    double z = 0;
    for(int state_to=0; state_to<x_dim*y_dim; ++ state_to) {
        z += raw_state_probability(state_from,action,state_to,x);
    }
    return z;
}

double KMarkovCRF::raw_reward_probability(const std::vector<int> & state_vector, const double& rew, lbfgsfloatval_t const * x) {
    if(x==nullptr) x = lambda_state;
    double prod = 1;
    for(uint f_idx=0; f_idx<reward_features.size(); ++f_idx) {
        prod *= exp( x[reward_parameter_indices[f_idx]] * reward_features[f_idx].evaluate(state_vector,rew) );
    }
    return prod;
}

double KMarkovCRF::reward_partition_function(const std::vector<int> & state_vector, lbfgsfloatval_t const * x) {
    if(x==nullptr) x = lambda_state;
    return raw_reward_probability(state_vector,0.0,x) + raw_reward_probability(state_vector,1.0,x);
}

KMarkovCRF::probability_t KMarkovCRF::raw_probability(const episode_t& episode, lbfgsfloatval_t const * x) {
    if(x==nullptr) x = lambda;
    double prod = 1;
    for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
        prod *= exp( x[parameter_indices[f_idx]] * features[f_idx].evaluate(episode) );
    }
    return prod;
}

KMarkovCRF::probability_t KMarkovCRF::partition_function(const episode_t& episode, lbfgsfloatval_t const * x) {
    if(x==nullptr) x = lambda;
    double z = 0;
    episode_t episode_copy = episode;
    for(state_t state=0; state<x_dim*y_dim; ++ state) {
        std::get<1>(episode_copy.back()) = state;
        for(reward_t reward=0.0; reward<=1.0; ++reward) {
            std::get<2>(episode_copy.back()) = reward;
            z += raw_probability(episode_copy,x);
        }
    }
    return z;
}
