#include "TemporallyExtendedModel.h"

#include "ConjunctiveAdjacency.h"
#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../optimization/LBFGS_Object.h"

#include <iomanip>

#define DEBUG_LEVEL 3
#include "../util/debug.h"

using util::Range;
using util::INVALID;

typedef TemporallyExtendedModel TEM;

TEM::TemporallyExtendedModel(std::shared_ptr<ConjunctiveAdjacency> N): N_plus(N) {}

TEM::probability_t TEM::get_prediction(const_instance_ptr_t,
                                       const action_ptr_t&,
                                       const observation_ptr_t&,
                                       const reward_ptr_t&) const {
    return 0;
}

void TEM::add_action_observation_reward_tripel(
    const action_ptr_t& action,
    const observation_ptr_t& observation,
    const reward_ptr_t& reward,
    const bool& new_episode
    ) {
    HistoryObserver::add_action_observation_reward_tripel(action,observation,reward,new_episode);
    data_up_to_date = false;
}

void TEM::optimize_weights() {
    DEBUG_OUT(2,"Optimize weights");
    double old_log_like = -DBL_MAX;
    double new_log_like = 0;
    int iteration_count = 0;
    while(true) {
        vec_t grad;
        new_log_like = log_likelihood(grad);
        weights = 1e-10*grad;
        DEBUG_OUT(1,"Iteration " << iteration_count << ": Likelihood = " << exp(new_log_like));
        DEBUG_OUT(1,"    old log-like: " << old_log_like);
        DEBUG_OUT(1,"    new log-like: " << new_log_like);
        DEBUG_OUT(1,"           delta: " << fabs(old_log_like-new_log_like));
        if(fabs(old_log_like-new_log_like)<1e-10) {
            break;
        } else {
            ++iteration_count;
            old_log_like=new_log_like;
        }
    }
}

void TEM::grow_feature_set() {
    DEBUG_OUT(2,"Grow feature set");
    // get new features
    feature_set_t extension_features = (*N_plus)(feature_set);
    // remember weights of old features
    weight_map_t old_weights = get_weight_map();
    // insert new features
    feature_set.insert(extension_features.begin(),extension_features.end());
    // transfer weights (or initialize to zero)
    apply_weight_map(old_weights);
    DEBUG_OUT(2,"DONE (" << old_weights.size() << " --> " << feature_set.size() << " features)");
    // need to update data
    data_up_to_date = false;
}

void TEM::shrink_feature_set() {
    DEBUG_OUT(2,"Shrink feature set");
    weight_map_t old_weights = get_weight_map();
    for(auto f_weight_pair : old_weights) {
        if(f_weight_pair.second==0) {
            feature_set.erase(f_weight_pair.first);
        }
    }
    apply_weight_map(old_weights);
    DEBUG_OUT(2,"DONE (" << old_weights.size() << " --> " << feature_set.size() << " features)");
    // need to update data
    data_up_to_date = false;
}

void TEM::set_feature_set(const feature_set_t& new_set) {
    feature_set = new_set;
    weights.zeros(feature_set.size());
    data_up_to_date = false;
}

void TEM::print_features() const {
    DEBUG_OUT(0,"Feature Set:");
    int f_idx = 0;
    for(f_ptr_t f : feature_set) {
        DEBUG_OUT(0,QString("    %1: [%2]	")
                  .arg(f_idx,4)
                  .arg(weights(f_idx),7,'f',3) <<
                  *f
            );
        ++f_idx;
    }
}

void TEM::print_training_data() const {
    int data_idx = 0;
    int episode_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        DEBUG_OUT(0,"Episode " << episode_idx);
        int instance_idx = 0;
        for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {
            DEBUG_OUT(0,data_idx << "    Instance " << instance_idx << ": " << ins);
            ++instance_idx;
            ++data_idx;
        }
        ++episode_idx;
    }
}

lbfgsfloatval_t TEM::LBFGS_objective(const lbfgsfloatval_t* par, lbfgsfloatval_t* grad) {
    int nr_vars = weights.size();
    vec_t w(nr_vars);
    for(int idx=0; idx<nr_vars; ++idx) {
        w(idx) = par[idx];
    }
    vec_t g;
    double log_like = log_likelihood(g,w);
    for(int idx=0; idx<nr_vars; ++idx) {
        grad[idx] = g(idx);
    }
    return log_like;
}

bool TEM::check_derivatives(const int& number_of_samples,
                            const double& range,
                            const double& delta,
                            const double& allowed_maximum_relative_deviation,
                            const double& minimum_gradient,
                            const bool use_current_values
    ) {

    // use LBFGS_Object
    LBFGS_Object lbfgs;

    // create object of type LBFGS_Object::objective_t
    using std::placeholders::_1;
    using std::placeholders::_2;
    std::function<lbfgsfloatval_t(const lbfgsfloatval_t*, lbfgsfloatval_t*)> objective;
    objective = std::bind(&TEM::LBFGS_objective, *this, _1, _2);

    // set all values
    int nr_vars = weights.size();
    lbfgs.set_objective(objective);
    lbfgs.set_number_of_variables(nr_vars);
    if(use_current_values) {
        lbfgsfloatval_t * values = lbfgs_malloc(nr_vars);
        for(int idx : Range(nr_vars)) {
            values[idx] = weights(idx);
        }
        lbfgs.set_variables(values);
    }
    return lbfgs.check_derivatives(number_of_samples,
                                   range,
                                   delta,
                                   allowed_maximum_relative_deviation,
                                   minimum_gradient,
                                   use_current_values);
}

TEM::weight_map_t TEM::get_weight_map() const {
    weight_map_t weight_map;
    int idx = 0;
    for(f_ptr_t f : feature_set) {
        weight_map[f] = weights(idx);
        ++idx;
    }
    return weight_map;
}

void TEM::apply_weight_map(weight_map_t weight_map) {
    weights.set_size(feature_set.size());
    int idx = 0;
    for(f_ptr_t f : feature_set) {
        auto it = weight_map.find(f);
        if(it==weight_map.end()) {
            weights(idx) = 0;
        } else {
            weights(idx) = it->second;
        }
        ++idx;
    }
}

void TEM::update_data() {
    DEBUG_OUT(2,"Update data");
    print_training_data();
    // check size of weight vector
    if(weights.size()!=feature_set.size()) {
        DEBUG_DEAD_LINE;
        weight_map_t old_weights = get_weight_map();
        weights.set_size(feature_set.size());
        apply_weight_map(old_weights);
    }
    // check matching number of data points
    if(data_up_to_date &&
       (F_matrices.size()!=number_of_data_points ||
        outcome_indices.size()!=number_of_data_points)
        ) {
        DEBUG_DEAD_LINE;
        data_up_to_date = false;
    }
    // update F-matrices and outcome indices
    if(!data_up_to_date) {

        // get dimensions
        int data_n = number_of_data_points;
        int feature_n = feature_set.size();
        int outcome_n = observation_space->space_size()*reward_space->space_size();

        // set vector size
        F_matrices.resize(data_n);
        outcome_indices.assign(data_n,-1);

        // for all data points
        int data_idx = 0;
        for(const_instance_ptr_t episode : instance_data) {
            for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {

                // set matrix size (initialize to zero, which should cost almost
                // nothing for sparse matrices)
                f_mat_t& matrix = F_matrices[data_idx];
                matrix.zeros(feature_n,outcome_n);
                DEBUG_OUT(3,"Resize matrix nr " << data_idx << " --> (" <<
                          F_matrices[data_idx].n_rows << "," << F_matrices[data_idx].n_cols <<
                          ")"
                    );

                // for all outcomes
                int outcome_idx = 0;
                for(observation_ptr_t obs : observation_space) {
                    for(reward_ptr_t rew : reward_space) {

                        // for all features
                        int feature_idx = 0;
                        for(f_ptr_t feature : feature_set) {

                            // set entry to 1 for non-zero features
                            if(feature->evaluate(ins->const_prev(),ins->action,obs,rew)!=0) {
                                matrix(feature_idx,outcome_idx) = 1;
                            }

                            ++feature_idx;
                        }

                        // set index for matching observation and reward
                        if(obs==ins->observation && rew==ins->reward) {
                            outcome_indices[data_idx] = outcome_idx;
                        }

                        ++outcome_idx;
                    }
                }

                // check if outcome index was set
                if(outcome_indices[data_idx]<0) {
                    DEBUG_DEAD_LINE;
                }

                ++data_idx;
            }
        }

        if(DEBUG_LEVEL>=3) {
            DEBUG_OUT(0,"Feature matrices:");
            int idx = 0;
            for(auto F : F_matrices) {
                DEBUG_OUT(0,"Nr.: " << idx);
                F.print();
                ++idx;
            }
        }

        data_up_to_date = true;
    }
}

double TEM::log_likelihood(vec_t& grad, const vec_t& w) {

    // make sure data are up to date
    update_data();

#warning why does simple exp prodoce 1x1 matrix??
#warning why are sparse matrices interpreted as 1x1 matrices??

    double obj = 0;
    grad.zeros(feature_set.size());
    for(int data_idx=0; data_idx<(int)number_of_data_points; ++data_idx) {

        // get matrix and outcome index
        const int outcome_idx = outcome_indices[data_idx];
        const f_mat_t& F = F_matrices[data_idx];
        // construct dense matrix TODO use sparse matrix
        arma::mat F_dense(F.n_rows,F.n_cols);
        F_dense.zeros();
        F_dense += F;
        DEBUG_OUT(3,F_matrices.size());
        DEBUG_OUT(3,F_matrices[data_idx]);

        // debug output
        if(DEBUG_LEVEL>=3) {
            DEBUG_OUT(0,"Data point " << data_idx << ", outcome index " << outcome_idx);
            F_dense.print("F-Matrix:");
            w.print("Weights:");
        }

        // interim variables
        vec_t lin = w.t()*F;
        vec_t exp_lin = arma::trunc_exp(lin);
        double z = sum(exp_lin);

        // debug output
        if(DEBUG_LEVEL>=3) {
            lin.print("lin:");
            exp_lin.print("exp_lin:");
        }

        // compute objective and gradient
        DEBUG_OUT(3,"Compute objective component");
        obj += lin(outcome_idx)-log(z);
        DEBUG_OUT(3,"Compute gradient component");
        grad += F.col(outcome_idx) - F_dense*exp_lin.t()/z;
        DEBUG_OUT(3,"DONE");
    }
    // divide by number of data points
    if(number_of_data_points>0) {
        obj /= number_of_data_points;
        grad /= number_of_data_points;
    }
    // return
    return obj;
}

double TEM::log_likelihood(vec_t& grad) {
    return log_likelihood(grad,weights);
}
