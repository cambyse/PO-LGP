#include "TemporallyExtendedModel.h"

#include "ConjunctiveAdjacency.h"
#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../util/ProgressBar.h"

#include <omp.h>
#define USE_OMP

#include <iomanip>

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#define DEBUG_STRING "TEM: "
#include "../util/debug.h"

using util::Range;
using util::INVALID;

using std::cout;
using std::endl;
using std::vector;
using std::map;
using std::make_tuple;
using std::dynamic_pointer_cast;

using arma::zeros;

typedef TemporallyExtendedModel TEM;

TEM::TemporallyExtendedModel(std::shared_ptr<ConjunctiveAdjacency> N):
    TemporallyExtendedFeatureLearner(N)
{
    // include only action, observation, and reward features for t=0
    N_plus->set_t_zero_features(ConjunctiveAdjacency::T_ZERO_FEATURES::OBSERVATION_REWARD);
    // set outcome to "observation-reward"
    set_outcome_type(OUTCOME_TYPE::OBSERVATION_REWARD);
}

TEM::probability_t TEM::get_prediction(const_instance_ptr_t ins,
                                       const action_ptr_t& action,
                                       const observation_ptr_t& observation,
                                       const reward_ptr_t& reward) const {
    int outcome_idx = 0;
    int matching_outcome_idx = -1;
    f_mat_t F_matrix = zeros<f_mat_t>(feature_set.size(),observation_space->space_size()*reward_space->space_size());
    for(observation_ptr_t obs : observation_space) {
        for(reward_ptr_t rew : reward_space) {
            int feature_idx = 0;
            for(f_ptr_t feature : feature_set) {
                if(feature->evaluate(ins,action,obs,rew)!=0) {
                    F_matrix(feature_idx,outcome_idx) = 1;
                }
                // increment
                ++feature_idx;
            }
            if(obs==observation && rew==reward) {
                matching_outcome_idx = outcome_idx;
            }
            // increment
            ++outcome_idx;
        }
    }
    if(matching_outcome_idx==-1) { DEBUG_DEAD_LINE; }
    const row_vec_t lin = weights.t()*F_matrix;
    const row_vec_t exp_lin = arma::exp(lin);
    const probability_t z = sum(exp_lin);
    const probability_t l = lin(matching_outcome_idx)-log(z);
    return exp(l);
}

TEM::probability_map_t TEM::get_prediction_map(const_instance_ptr_t ins,
                                               const action_ptr_t& action) const {
    // compute feature matrix
    f_mat_t F_matrix = zeros<f_mat_t>(feature_set.size(),observation_space->space_size()*reward_space->space_size());
    {
        int outcome_idx = 0;
        for(observation_ptr_t obs : observation_space) {
            for(reward_ptr_t rew : reward_space) {
                int feature_idx = 0;
                for(f_ptr_t feature : feature_set) {
                    if(feature->evaluate(ins,action,obs,rew)!=0) {
                        F_matrix(feature_idx,outcome_idx) = 1;
                    }
                    // increment
                    ++feature_idx;
                }
                // increment
                ++outcome_idx;
            }
        }
    }

    // compute normalization
    const row_vec_t lin = weights.t()*F_matrix;
    const row_vec_t exp_lin = arma::exp(lin);
    const probability_t log_z = log(sum(exp_lin));

    // fill probability map
    DEBUG_OUT(3,"Predictions for " << ins << " / " << action);
    probability_map_t return_map;
    {
        int outcome_idx = 0;
        for(observation_ptr_t obs : observation_space) {
            for(reward_ptr_t rew : reward_space) {
                // compute probability
                const probability_t l = lin(outcome_idx)-log_z;
                return_map[make_tuple(obs,rew)] = exp(l);
                DEBUG_OUT(3,"    p(" << obs << "," << rew << "): " << exp(l));
                // increment
                ++outcome_idx;
            }
        }
    }

    // return
    return return_map;
}

TEM::probability_t TEM::neg_log_likelihood(col_vec_t& grad, const col_vec_t& w) {

    DEBUG_OUT(3,"Compute neg-log-likelihood");

    // check dimensions of input
    if(DEBUG_LEVEL>0) {
        if(grad.size()!=weights.size()){
            DEBUG_DEAD_LINE;
        }
        if(w.size()!=weights.size()) {
            DEBUG_DEAD_LINE;
        }
    }

    // initialize objective and gradient
    probability_t obj = 0;
    grad.zeros(feature_set.size());

#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(1)
#endif

    // sum over data
    for(int data_idx=0; data_idx<(int)number_of_data_points; ++data_idx) {

        // get matrix and outcome index
        const int outcome_idx = outcome_indices[data_idx];
        const f_mat_t& F = F_matrices[data_idx];

        // debug output
        if(DEBUG_LEVEL>=3) {
            DEBUG_OUT(0,"Data point " << data_idx << ", outcome index " << outcome_idx);
            F.print("Sparse F-Matrix");
            w.print("Weights:");
        }

        // interim variables
        const row_vec_t lin = w.t()*F;
        const row_vec_t exp_lin = arma::exp(lin);
        const probability_t z = sum(exp_lin);

        // debug output
        if(DEBUG_LEVEL>=3) {
            lin.print("lin:");
            exp_lin.print("exp_lin:");
        }

        // compute objective and gradient
        probability_t obj_comp = lin(outcome_idx)-log(z);
        col_vec_t grad_comp = F.col(outcome_idx) - F*exp_lin.t()/z;

#ifdef USE_OMP
#pragma omp critical (TemporallyExtendedModel)
#pragma omp critical (TemporallyExtendedFeatureLearner)
#endif
        {
            // update
            obj += obj_comp;
            grad += grad_comp;
            if(DEBUG_LEVEL>=3) {
                grad_comp.print("gradient component:");
                DEBUG_OUT(0,"objective component " << obj_comp);
            }
        } // end critical
    } // end parallel

    // divide by number of data points and reverse sign
    if(number_of_data_points>0) {
        obj = -obj/number_of_data_points;
        grad = -grad/number_of_data_points;
    }

    if(DEBUG_LEVEL>=3) {
        DEBUG_OUT(0,"    Objective value: " << obj);
        DEBUG_OUT(0,"    Parameters	Gradient: ");
        for(int idx : Range(w.n_rows)) {
            DEBUG_OUT(0,"    " << w(idx) << "	" << grad(idx));
        }
    }

    // increment counter
    ++objective_evaluations;

    // return
    return obj;

}

lbfgsfloatval_t TEM::LBFGS_objective(const lbfgsfloatval_t* par, lbfgsfloatval_t* grad) {
    int nr_vars = weights.size();
    col_vec_t w(par,nr_vars);
    col_vec_t g(grad,nr_vars,false);
    probability_t neg_log_like = neg_log_likelihood(g,w);
    return neg_log_like;
}

int TEM::LBFGS_progress(const lbfgsfloatval_t * x,
                        const lbfgsfloatval_t */*g*/,
                        const lbfgsfloatval_t fx,
                        const lbfgsfloatval_t /*xnorm*/,
                        const lbfgsfloatval_t /*gnorm*/,
                        const lbfgsfloatval_t /*step*/,
                        int nr_variables,
                        int iteration_nr,
                        int /*ls*/) const {
    IF_DEBUG(1) {
        // L1 norm //
        lbfgsfloatval_t xnorm = L1_norm(x, nr_variables);
        cout <<
            QString("    Iteration %1 (%2), Likelihood + L1 = %3 + %4")
            .arg(iteration_nr)
            .arg(objective_evaluations)
            .arg(exp(-(fx-xnorm*l1_factor)),7,'f',5)
            .arg(xnorm*l1_factor,7,'f',5)
             << endl;
    }
    // DEBUG_OUT(1,"Iteration " << iteration_nr << " (" << objective_evaluations << "), Likelihood = " << exp(-fx));
    return 0;
}

void TEM::LBFGS_final_message(probability_t obj_val) const {
    IF_DEBUG(1) {
        // L1 norm //
        lbfgsfloatval_t xnorm = arma::as_scalar(arma::sum(arma::abs(weights)));
        cout <<
            QString("    Likelihood + L1 = %1 + %2")
            .arg(exp(-(obj_val-xnorm*l1_factor)),7,'f',5)
            .arg(xnorm*l1_factor,7,'f',5)
             << endl;
    }
    // DEBUG_OUT(0,"Likelihood = " << exp(-obj_val));
}
