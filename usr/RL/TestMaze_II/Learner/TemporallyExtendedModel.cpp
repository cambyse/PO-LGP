#include "TemporallyExtendedModel.h"

#include "ConjunctiveAdjacency.h"
#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../util/ProgressBar.h"

#include <omp.h>
//#define USE_OMP

#include <iomanip>

#define DEBUG_LEVEL 2
#include "../util/debug.h"

using util::Range;
using util::INVALID;

using std::vector;
using std::dynamic_pointer_cast;

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
    data_changed = true;
}

// void TEM::optimize_weights_SGD() {
//     DEBUG_OUT(2,"Optimize weights using SGD");
//     double old_neg_log_like = -DBL_MAX;
//     double new_neg_log_like = 0;
//     int iteration_count = 0;
//     while(true) {
//         double alpha = 1e-2; // learning rate
//         vec_t grad;
//         new_neg_log_like = neg_log_likelihood(grad,weights);
//         weights = weights - alpha*grad;
//         DEBUG_OUT(1,"Iteration " << iteration_count << ": Likelihood = " << exp(-new_neg_log_like));
//         DEBUG_OUT(1,"    old neg-log-like: " << old_neg_log_like);
//         DEBUG_OUT(1,"    new neg-log-like: " << new_neg_log_like);
//         DEBUG_OUT(1,"               delta: " << fabs(old_neg_log_like-new_neg_log_like));
//         if(iteration_count>10 && fabs(old_neg_log_like-new_neg_log_like)<1e-5) {
//             break;
//         } else {
//             ++iteration_count;
//             old_neg_log_like=new_neg_log_like;
//         }
//     }
// }

void TEM::optimize_weights_LBFGS() {

    DEBUG_OUT(2,"Optimize weights using L-BFGS");

    // dimension
    int nr_vars = weights.size();
    vector<lbfgsfloatval_t> values(weights.begin(),weights.end());

    // use LBFGS_Object
    LBFGS_Object lbfgs;
    lbfgs.set_objective(get_LBFGS_objective());
    lbfgs.set_progress(get_LBFGS_progress());
    lbfgs.set_number_of_variables(nr_vars);
    lbfgs.set_variables(values);
    lbfgs.set_l1_factor(l1_factor);
    double neg_log_like = lbfgs.optimize(values);
    DEBUG_OUT(1,"Likelihood = " << exp(-neg_log_like));

    // set weights
    weights = values;
}

void TEM::grow_feature_set() {
    DEBUG_OUT(2,"Grow feature set");
    // get new features
    f_set_t extension_features = (*N_plus)(feature_set);
    // remember weights of old features
    weight_map_t old_weights = get_weight_map();
    // insert new features
    feature_set.insert(extension_features.begin(),extension_features.end());
    // transfer weights (or initialize to zero)
    apply_weight_map(old_weights);
    DEBUG_OUT(2,"DONE (" << old_weights.size() << " --> " << feature_set.size() << " features)");
    // need to update data
    feature_set_changed = true;
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
    feature_set_changed = true;
}

void TEM::set_feature_set(const f_set_t& new_set) {
    feature_set = new_set;
    weights.zeros(feature_set.size());
    feature_set_changed = true;
}

void TEM::set_l1_factor(const double& l1) {
    l1_factor = l1;
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

bool TEM::check_derivatives(const int& number_of_samples,
                            const double& range,
                            const double& delta,
                            const double& allowed_maximum_relative_deviation,
                            const double& minimum_gradient,
                            const bool use_current_values
    ) {

    // use LBFGS_Object
    LBFGS_Object lbfgs;

    // set all values
    int nr_vars = weights.size();
    lbfgs.set_objective(get_LBFGS_objective());
    lbfgs.set_number_of_variables(nr_vars);
    if(use_current_values) {
        vector<lbfgsfloatval_t> values(weights.begin(),weights.end());
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

void TEM::update() {

    DEBUG_OUT(3,"Check if everything is up to date");

    //---------------------------------------------//
    // some sanity checks for non-zero debug level //
    //---------------------------------------------//

    // check size of weight vector
    if(DEBUG_LEVEL>0 && weights.size()!=feature_set.size()) {
        DEBUG_DEAD_LINE;
        weight_map_t old_weights = get_weight_map();
        weights.set_size(feature_set.size());
        apply_weight_map(old_weights);
    }
    // check matching number of data points
    if(DEBUG_LEVEL>0 && !data_changed &&
       (F_matrices.size()!=number_of_data_points ||
        outcome_indices.size()!=number_of_data_points )) {
        DEBUG_DEAD_LINE;
        data_changed = true;
    }
    // check dimensions of F-matrices (check one for all)
    if(DEBUG_LEVEL>0 && !feature_set_changed && number_of_data_points>0 &&
       F_matrices.back().n_rows!=feature_set.size()) {
        DEBUG_DEAD_LINE;
        feature_set_changed = true;
    }

    // check if anything needs to be updated
    if(data_changed || feature_set_changed) {
        int data_n = number_of_data_points;
        int feature_n = feature_set.size();
        int outcome_n = observation_space->space_size()*reward_space->space_size();
        DEBUG_OUT(2,"Update for:");
        DEBUG_OUT(2,"    nr data points: " << number_of_data_points);
        DEBUG_OUT(2,"       nr features: " << feature_n);
        DEBUG_OUT(2,"       nr outcomes: " << outcome_n);

        //-----------------------//
        // update basis features //
        //-----------------------//
        bool basis_features_changed = false;
        if(feature_set_changed) {
            DEBUG_OUT(2,"Update basis feature set");
            f_ptr_set_t new_basis_features;
            for(f_ptr_t feature : feature_set) {
                auto and_feature = dynamic_pointer_cast<const AndFeature>(feature);
                if(and_feature!=nullptr) {
                    auto subfeatures = and_feature->get_subfeatures();
                    new_basis_features.insert(subfeatures.begin(),subfeatures.end());
                } else {
                    DEBUG_DEAD_LINE;
                }
            }
            if(new_basis_features!=basis_features) {
                basis_features = new_basis_features;
                basis_features_changed = true;
            }
        }

        //----------------------------//
        // other things to be updated //
        //----------------------------//
        bool update_basis_feature_map = basis_features_changed || data_changed;
        bool update_F_matrices = data_changed || feature_set_changed;
        bool update_outcome_indices = data_changed;

        // set vectors size
        if(update_basis_feature_map) {
            DEBUG_OUT(2,"Update basis feature map");
            basis_feature_map.assign(data_n,vector<basis_feature_map_t>(outcome_n));
        }
        if(update_F_matrices) {
            DEBUG_OUT(2,"Update F-matrices (" << feature_n << "," << outcome_n << ")");
            #warning need to clear for correct dimensions -- why? --> write minimal example
            F_matrices.clear();
            F_matrices.resize(data_n,f_mat_t(feature_n,outcome_n));
        }
        if(update_outcome_indices) {
            DEBUG_OUT(2,"Update outcome indices");
            outcome_indices.assign(data_n,-1);
        }

        // for all data points
        int data_idx = 0;
        if(DEBUG_LEVEL>0) {ProgressBar::init("Updating: ");}
        for(const_instance_ptr_t episode : instance_data) {
            for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {

                // get F-matrix for this data point
                f_mat_t& F_matrix = F_matrices[data_idx];

                // for all outcomes
                int outcome_idx = 0;
                for(observation_ptr_t obs : observation_space) {
                    for(reward_ptr_t rew : reward_space) {

                        // get basis feature map for this data point and outcome
                        basis_feature_map_t& bf_map = basis_feature_map[data_idx][outcome_idx];

                        //--------------------------//
                        // update basis feature map //
                        //--------------------------//
                        if(update_basis_feature_map) {
                            bf_map.clear();
                            for(f_ptr_t bf : basis_features) {
                                bf_map.insert_feature(bf,bf->evaluate(ins->const_prev(),ins->action,obs,rew));
                            }
                        }

                        //-----------------//
                        // update F-matrix //
                        //-----------------//
                        if(update_F_matrices) {
                            int feature_idx = 0;
                            for(f_ptr_t feature : feature_set) {

                                // set entry to 1 for non-zero features
                                if(feature->evaluate(ins->const_prev(),ins->action,obs,rew)!=0) {
                                    #warning use map but does not work for now
                                    //if(feature->evaluate(bf_map)) {
                                    DEBUG_OUT(4,"(" << F_matrix.n_rows << "," << F_matrix.n_cols << ")/(" << feature_idx << "," << outcome_idx << ")");
                                    F_matrix(feature_idx,outcome_idx) = 1;
                                }

                                // increment
                                ++feature_idx;
                            }
                        }

                        //----------------------//
                        // update outcome index //
                        //----------------------//
                        if(update_outcome_indices && obs==ins->observation && rew==ins->reward) {
                            outcome_indices[data_idx] = outcome_idx;
                        }

                        // increment
                        ++outcome_idx;
                    }
                }

                // check if outcome index was set
                if(update_outcome_indices && outcome_indices[data_idx]<0) {
                    DEBUG_DEAD_LINE;
                }

                // print progress
                if(DEBUG_LEVEL>0) {ProgressBar::print(data_idx,number_of_data_points);}

                // increment
                ++data_idx;
            }
        }

        // terminate progress
        if(DEBUG_LEVEL>0) {ProgressBar::terminate();}

        // data and features are up to date
        data_changed = false;
        feature_set_changed = false;

        // print F-matrices
        if(DEBUG_LEVEL>=3) {
            DEBUG_OUT(0,"Feature matrices:");
            int idx = 0;
            for(auto F : F_matrices) {
                DEBUG_OUT(0,"Nr.: " << idx);
                F.print();
                ++idx;
            }
        }
    }
}

double TEM::neg_log_likelihood(vec_t& grad, const vec_t& w) {

    /** Some issues with armadillo:
     *
     * -- simple exp(vector) prodoces 1x1 matrix, trunc_exp does not -- don't
          know why
     *
     * -- SpMat type does not mix well with other types, e.g., F*exp_lin.t()
     * below produces a 1x1 matrix if F is sparse but not if F_dense is used. */

#warning todo: write a minimal example for this behavior

    DEBUG_OUT(3,"Compute neg-log-likelihood");

    // make sure data are up to date
    update();

    // initialize objective and gradient
    double obj = 0;
    grad.zeros(feature_set.size());

    // sum over data
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
        const vec_t lin = w.t()*F;
        const vec_t exp_lin = arma::trunc_exp(lin);
        const double z = sum(exp_lin);

        // debug output
        if(DEBUG_LEVEL>=3) {
            lin.print("lin:");
            exp_lin.print("exp_lin:");
        }

        // compute objective and gradient
        if(DEBUG_LEVEL>=3) {
            double obj_comp = lin(outcome_idx)-log(z);
            vec_t grad_comp = F.col(outcome_idx) - F_dense*exp_lin.t()/z;
            obj += obj_comp;
            grad += grad_comp;
            grad_comp.print("gradient component:");
            DEBUG_OUT(0,"objective component " << obj_comp);
        } else {
            obj += lin(outcome_idx)-log(z);
            grad += F.col(outcome_idx) - F_dense*exp_lin.t()/z;
        }
    }

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

    // return
    return obj;
}

lbfgsfloatval_t TEM::LBFGS_objective(const lbfgsfloatval_t* par, lbfgsfloatval_t* grad) {
    int nr_vars = weights.size();
    vec_t w(par,nr_vars);
    vec_t g(grad,nr_vars,false);
    double neg_log_like = neg_log_likelihood(g,w);
    return neg_log_like;
}

int TEM::LBFGS_progress(const lbfgsfloatval_t */*x*/,
                        const lbfgsfloatval_t */*g*/,
                        const lbfgsfloatval_t fx,
                        const lbfgsfloatval_t /*xnorm*/,
                        const lbfgsfloatval_t /*gnorm*/,
                        const lbfgsfloatval_t /*step*/,
                        int /*nr_variables*/,
                        int iteration_nr,
                        int /*ls*/) const {
    DEBUG_OUT(1,"Iteration " << iteration_nr << " (Likelihood = " << exp(-fx) << ")" );
    //for(int x_idx : Range(nr_variables)) {
        //DEBUG_OUT(1, "    x[" << x_idx << "] = " << x[x_idx]);
    //}
    //DEBUG_OUT(1,"Iteration " << iteration_nr << " (Likelihood = " << exp(-fx) << ")" );
    return 0;
}

LBFGS_Object::objective_t TEM::get_LBFGS_objective() {
    return std::bind(&TEM::LBFGS_objective, this, std::placeholders::_1, std::placeholders::_2);
}

LBFGS_Object::progress_t TEM::get_LBFGS_progress() const {
    return std::bind(&TEM::LBFGS_progress, this,
                     std::placeholders::_1,
                     std::placeholders::_2,
                     std::placeholders::_3,
                     std::placeholders::_4,
                     std::placeholders::_5,
                     std::placeholders::_6,
                     std::placeholders::_7,
                     std::placeholders::_8,
                     std::placeholders::_9
        );
}
