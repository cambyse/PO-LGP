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

TELQ::action_ptr_t TELQ::get_action(const_instance_ptr_t) {
    return action_space;
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
        double max_action_value = -DBL_MAX;
        for(action_ptr_t act : action_space) {
            if(action_values(outcome_idx)>max_action_value) {
                max_action_value = action_values(outcome_idx);
                optimal_actions.resize(1,act);
            } else if(action_values(outcome_idx)==max_action_value) {
                optimal_actions.push_back(act);
            }
            ++outcome_idx;
        }
        assert(optimal_actions.size()>0);
        policy[data_idx] = util::random_select(optimal_actions);
    }
}

void TELQ::update_objective_components() {
    c = 0;
    rho.zeros(feature_set.size());
    L.zeros(feature_set.size(),feature_set.size());
    int data_idx = 0;
    int normalization = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins=episode->const_first(); ins->const_next()!=INVALID; ++ins) {
            double r_t = ins->reward->get_value();
            col_vec_t vec1 = F_matrices[data_idx+1].col(policy_indices[data_idx+1]);
            col_vec_t vec2 = F_matrices[data_idx  ].col(outcome_indices[data_idx]);
            col_vec_t vec3 = discount * vec1 - vec2;
            c += pow(r_t,2);
            rho += r_t * vec3;
            L += arma::kron(vec3,vec3.t());
            ++data_idx;
            ++normalization;
        }
        ++data_idx; // last instance is skipped in inner loop
    }
    assert(data_idx==number_of_data_points);
    c /= normalization;
    rho /= normalization;
    L /= normalization;
}

lbfgsfloatval_t TELQ::LBFGS_objective(const lbfgsfloatval_t* par, lbfgsfloatval_t* grad) {
    int nr_vars = weights.size();
    col_vec_t w(par,nr_vars);
    col_vec_t g(grad,nr_vars,false);
    double neg_log_like;// = neg_log_likelihood(g,w);
    return neg_log_like;
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
    DEBUG_OUT(1,"Iteration " << iteration_nr << " (" << objective_evaluations << "), Likelihood = " << exp(-fx));
    return 0;
}
