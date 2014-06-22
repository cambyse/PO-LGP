#include "TemporallyExtendedLinearQ.h"

#include "ConjunctiveAdjacency.h"
#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../util/ProgressBar.h"

#include <omp.h>
#define USE_OMP

#include <iomanip>

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using util::Range;
using util::INVALID;

using std::vector;
using std::map;
using std::make_tuple;
using std::dynamic_pointer_cast;

typedef TemporallyExtendedLinearQ TELQ;

TemporallyExtendedLinearQ::TemporallyExtendedLinearQ(std::shared_ptr<ConjunctiveAdjacency> N,
                                                     double d):
    TemporallyExtendedFeatureLearner(N), discount(d)
{
    // include only action features for t=0
    N_plus->set_t_zero_features(ConjunctiveAdjacency::ACTION);
    // set outcome to "action"
    set_outcome_type(OUTCOME_TYPE::ACTION);
}

TELQ::action_ptr_t TELQ::get_action(const_instance_ptr_t ins) {
    // print random action if values cannot be computed
    if(feature_set.size()==0) {
        DEBUG_WARNING("Cannot compute action (no features)");
        return action_space->random_element();
    }
    vector<action_ptr_t> optimal_actions;
    double max_action_value = -DBL_MAX;
    for(action_ptr_t act : action_space) {
        // get feature values
        row_vec_t feature_values(feature_set.size());
        int feature_idx = 0;
        for(f_ptr_t feature : feature_set) {
            feature_values(feature_idx) = feature->evaluate(ins,act,observation_space,reward_space);
            ++feature_idx;
        }
        // compute action value
        double action_value = arma::as_scalar(feature_values*weights);
        // update optimal actions
        if(action_value>max_action_value) {
            max_action_value = action_value;
            optimal_actions.resize(1,act);
        } else if(action_value==max_action_value) {
            optimal_actions.push_back(act);
        }
    }
    assert(optimal_actions.size()>0);
    int random_idx = rand()%optimal_actions.size();
    return optimal_actions[random_idx];
}

double TELQ::run_policy_iteration() {
    int counter = 0;
    double old_TD_error = DBL_MAX;
    update_policy();
    update_c_rho_L();
    enum MODE { ANALYTICALLY, L1 };
    MODE mode = ANALYTICALLY;
    if(DEBUG_LEVEL>0) {ProgressBar::init("Policy Iteration:          ");}
    while(true) {
        update_c_rho_L();
        switch(mode) {
        case ANALYTICALLY: optimize_weights_analytically();
            break;
        case L1: optimize_weights_L1();
            break;
        }
        update_policy();
        double TD_error = arma::as_scalar(c + 2*rho.t()*weights + weights.t()*L*weights);
        ++counter;
        IF_DEBUG(1) {
            if(mode==ANALYTICALLY) {
                ProgressBar::msg() << " (" << counter << "/" << TD_error << ")";
                ProgressBar::print(counter%2,1);
            } else {
                DEBUG_OUT(0,"Iteration: " << counter << " / TD-error: " << TD_error);
            }
        }
        if(old_TD_error!=TD_error) {
            old_TD_error = TD_error;
        } else {
            if(mode==ANALYTICALLY) {
                if(DEBUG_LEVEL>0) ProgressBar::terminate();
                mode = L1;
            } else {
                break;
            }
        }
    }
    return old_TD_error;
}

void TELQ::update_policy() {
    // make sure all data are up-to-date
    update();
    // resize
    policy.resize(number_of_data_points);
    policy_indices.resize(number_of_data_points);
    // random policy if value function cannot be computed
    if(weights.size()==0) {
        for(int data_idx : Range(number_of_data_points)) {
            policy[data_idx] = action_space->random_element();
            unsigned int action_idx = 0;
            for(action_ptr_t act_compare : action_space) {
                if(act_compare==policy[data_idx]) {
                    policy_indices[data_idx] = action_idx;
                    break;
                }
                ++action_idx;
            }
            assert(action_idx<action_space->space_size());
        }
        return;
    }
    // choose maximum value action
    for(int data_idx : Range(number_of_data_points)) {
        row_vec_t action_values = weights.t()*F_matrices[data_idx];
        int outcome_idx = 0;
        vector<action_ptr_t> optimal_actions;
        vector<int> optimal_action_indices;
        double max_action_value = -DBL_MAX;
        for(action_ptr_t act : action_space) {
            if(action_values(outcome_idx)>max_action_value) {
                max_action_value = action_values(outcome_idx);
                optimal_actions.resize(1,act);
                optimal_action_indices.resize(1,outcome_idx);
            } else if(action_values(outcome_idx)==max_action_value) {
                optimal_actions.push_back(act);
                optimal_action_indices.push_back(outcome_idx);
            }
            ++outcome_idx;
        }
        assert(optimal_actions.size()>0);
        int random_idx = rand()%optimal_actions.size();
        policy[data_idx] = optimal_actions[random_idx];
        policy_indices[data_idx] = optimal_action_indices[random_idx];
    }
}

void TELQ::update_c_rho_L() {
    c = 0;
    rho.zeros(feature_set.size());
    L.zeros(feature_set.size(),feature_set.size());
    int data_idx = 0;
    int normalization = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins=episode->const_first(); ins->const_next()!=INVALID; ++ins) {
            double r_t = ins->reward->get_value();
            col_vec_t vec1 = (col_vec_t)F_matrices[data_idx+1].col(policy_indices[data_idx+1]);
            col_vec_t vec2 = (col_vec_t)F_matrices[data_idx  ].col(outcome_indices[data_idx]);
            col_vec_t vec3 = discount * vec1 - vec2;
            c = c + pow(r_t,2);
            rho = rho + r_t * vec3;
            L = L + arma::kron(vec3,vec3.t());
            ++data_idx;
            ++normalization;
        }
        ++data_idx; // last instance is skipped in inner loop
    }
    assert(data_idx==(int)number_of_data_points);
    c /= normalization;
    rho /= normalization;
    L /= normalization;
}

void TELQ::optimize_weights_analytically() {
    // use small regularization to make solution unique
    L = L + 1e-10*arma::eye(L.n_rows,L.n_cols);
    weights = arma::solve((arma::mat)L,-1*rho);
    L = L - 1e-10*arma::eye(L.n_rows,L.n_cols);
}

double TELQ::objective_and_gradient(col_vec_t& grad, const col_vec_t& weights) {
    grad = 2*rho + 2*L*weights;
    return arma::as_scalar(c + 2*rho.t()*weights + weights.t()*L*weights);
}

lbfgsfloatval_t TELQ::LBFGS_objective(const lbfgsfloatval_t* par, lbfgsfloatval_t* grad) {
    update();
    update_c_rho_L();
    int nr_vars = weights.size();
    col_vec_t w(par,nr_vars);
    col_vec_t g(grad,nr_vars,false);
    double TD_error = objective_and_gradient(g,w);
    ++objective_evaluations;
    return TD_error;
}

int TELQ::LBFGS_progress(const lbfgsfloatval_t */*x*/,
                        const lbfgsfloatval_t */*g*/,
                        const lbfgsfloatval_t fx,
                        const lbfgsfloatval_t /*xnorm*/,
                        const lbfgsfloatval_t /*gnorm*/,
                        const lbfgsfloatval_t /*step*/,
                        int /*nr_variables*/,
                        int iteration_nr,
                        int /*ls*/) const {
    DEBUG_OUT(1,"Iteration " << iteration_nr << " (" << objective_evaluations << "), TD-error+L1 = " << fx);
    return 0;
}

void TELQ::LBFGS_final_message(double obj_val) const {
    DEBUG_OUT(0,"TD-error+L1 = " << obj_val);
}
