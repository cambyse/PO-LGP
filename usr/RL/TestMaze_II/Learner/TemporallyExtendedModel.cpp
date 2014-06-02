#include "TemporallyExtendedModel.h"

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

typedef TemporallyExtendedModel TEM;

TEM::TemporallyExtendedModel(std::shared_ptr<ConjunctiveAdjacency> N): N_plus(N) {}

TEM::probability_t TEM::get_prediction(const_instance_ptr_t ins,
                                       const action_ptr_t& action,
                                       const observation_ptr_t& observation,
                                       const reward_ptr_t& reward) const {
    int outcome_idx = 0;
    int matching_outcome_idx = -1;
    f_mat_t F_matrix(feature_set.size(),observation_space->space_size()*reward_space->space_size());
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
    const row_vec_t exp_lin = arma::trunc_exp(lin);
    const double z = sum(exp_lin);
    const double l = lin(matching_outcome_idx)-log(z);
    return exp(l);
}

TEM::probability_map_t TEM::get_prediction_map(const_instance_ptr_t ins,
                                               const action_ptr_t& action) const {
    // compute feature matrix
    f_mat_t F_matrix(feature_set.size(),observation_space->space_size()*reward_space->space_size());
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
    const row_vec_t exp_lin = arma::trunc_exp(lin);
    const double log_z = log(sum(exp_lin));

    // fill probability map
    DEBUG_OUT(3,"Predictions for " << ins << " / " << action);
    probability_map_t return_map;
    {
        int outcome_idx = 0;
        for(observation_ptr_t obs : observation_space) {
            for(reward_ptr_t rew : reward_space) {
                // compute probability
                const double l = lin(outcome_idx)-log_z;
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
//     update();
//     DEBUG_OUT(2,"Optimize weights using SGD");
//     double old_neg_log_like = -DBL_MAX;
//     double new_neg_log_like = 0;
//     int iteration_count = 0;
//     while(true) {
//         double alpha = 1e-2; // learning rate
//         col_vec_t grad;
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

    // return if no data available
    if(number_of_data_points==0) {
        DEBUG_WARNING("Cannot optimize weights without data");
        return;
    }

    // make sure data are up to date
    update();

    // dimension
    int nr_vars = weights.size();
    vector<lbfgsfloatval_t> values(weights.begin(),weights.end());

    // reset number of objective evaluations
    objective_evaluations = 0;

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
    // use progress for unified output
    if(DEBUG_LEVEL>0) {ProgressBar::init("Grow feature set:          ");}
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
    // terminate progress
    if(DEBUG_LEVEL>0) {
        ProgressBar::msg() << " (" << old_weights.size() << " --> " << feature_set.size() << ")";
        ProgressBar::terminate();
    }
}

void TEM::shrink_feature_set() {
    // use progress for unified output
    if(DEBUG_LEVEL>0) {ProgressBar::init("Shrink feature set:        ");}
    weight_map_t old_weights = get_weight_map();
    for(auto f_weight_pair : old_weights) {
        if(f_weight_pair.second==0) {
            feature_set.erase(f_weight_pair.first);
        }
    }
    apply_weight_map(old_weights);
    // need to update data
    feature_set_changed = true;
    // terminate progress
    if(DEBUG_LEVEL>0) {
        ProgressBar::msg() << " (" << old_weights.size() << " --> " << feature_set.size() << ")";
        ProgressBar::terminate();
    }
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

    // make sure data are up to date
    update();

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

void TEM::clear_data() {
    HistoryObserver::clear_data();
    data_changed = true;
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
    if(DEBUG_LEVEL>0) {
        // check size of weight vector
        if(weights.size()!=feature_set.size()) {
            DEBUG_DEAD_LINE;
            weight_map_t old_weights = get_weight_map();
            weights.set_size(feature_set.size());
            apply_weight_map(old_weights);
        }
        // check matching number of data points
        if(!data_changed &&
           (F_matrices.size()!=number_of_data_points || outcome_indices.size()!=number_of_data_points )) {
            DEBUG_DEAD_LINE;
            data_changed = true;
        }
        // check dimensions of F-matrices (check one for all)
        if(!feature_set_changed && number_of_data_points>0 &&
           F_matrices.back().n_rows!=feature_set.size()) {
            DEBUG_DEAD_LINE;
            feature_set_changed = true;
        }
    }

    // check if anything needs to be updated
    if(data_changed || feature_set_changed) {

        // update basis features
        bool basis_features_changed = false;
        if(feature_set_changed) {
            basis_features_changed = update_basis_features();
        }

        // update basis feature maps
        if(data_changed || basis_features_changed) {
            update_basis_feature_maps(data_changed);
        }

        // pick non-const features
        bool feature_set_changed_again = false;
        if(feature_set_changed) {
            feature_set_changed_again = pick_non_const_features();
        }

        // again update basis features and maps
        if(feature_set_changed_again) {

            // update basis features again
            bool basis_features_changed_again = update_basis_features();

            // update basis feature maps
            if(basis_features_changed_again) {
                update_basis_feature_maps(false);
            }
        }

        // update F-matrices
        if(data_changed || feature_set_changed) {
            update_F_matrices();
        }

        // update outcome indices
        if(data_changed) {
            update_outcome_indices();
        }

        // data and features are up to date now
        data_changed = false;
        feature_set_changed = false;
    }
}

bool TEM::pick_non_const_features() {

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
    // Caution: basis_feature_maps must be up to date //
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

    if(DEBUG_LEVEL>0) {ProgressBar::init("Pick non-const features:   ");}

    // map for storing the first return-value for every outcome and feature
    typedef map<f_ptr_t,f_ret_t> f_ret_map_t;
    f_ret_map_t f_ret_map;

    // set of features that may be const
    f_set_t maybe_const_set = feature_set; // why f_set_t and not faster f_ptr_set_t?

    // initialize
    {
        int data_idx = 1;
        const_instance_ptr_t ins=instance_data.front()->const_first();

        // non-const features
        f_set_t non_const_set;

        // for all outcomes
        int outcome_idx = 0;
        for(observation_ptr_t obs : observation_space) {
            for(reward_ptr_t rew : reward_space) {

                // for all features that may be const
                for(f_ptr_t feature : maybe_const_set) {
                    // initialize on first data point
                    f_ret_map[feature] = feature->evaluate(basis_feature_maps[data_idx][outcome_idx]);
                }

                // increment
                ++outcome_idx;
            }
        }

        // erase non-const features from maybe-const set and return-value
        // maps
        for(f_ptr_t feature : non_const_set) {
            DEBUG_OUT(3,"    non-const: " << *feature);
            auto it_maybe = maybe_const_set.find(feature);
            if(it_maybe!=maybe_const_set.end()) {
                maybe_const_set.erase(it_maybe);
            } else {
                DEBUG_DEAD_LINE;
            }
            auto it = f_ret_map.find(feature);
            if(it!=f_ret_map.end()) {
                f_ret_map.erase(it);
            } else {
                DEBUG_DEAD_LINE;
            }
        }
    }

    // for all data points
#ifdef USE_OMP
    int progress_idx = 0;
#pragma omp parallel for schedule(static) collapse(1)
    for(int data_idx = 0; data_idx<(int)number_of_data_points; ++data_idx) {
#else
        int data_idx = 0;
        int& progress_idx = data_idx;
        for(const_instance_ptr_t episode : instance_data) {
            for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {
#endif

                // non-const features
                f_set_t non_const_set;

                // for all outcomes
                int outcome_idx = 0;
                for(observation_ptr_t obs : observation_space) {
                    for(reward_ptr_t rew : reward_space) {

                        // for all features that may be const
#ifdef USE_OMP
                        f_set_t maybe_const_set_copy;
                        f_ret_map_t f_ret_map_copy;
#pragma omp critical
                        {
                            maybe_const_set_copy = maybe_const_set;
                            f_ret_map_copy = f_ret_map;

                        } // critical
                        for(f_ptr_t feature : maybe_const_set_copy) {
#else
                            f_set_t& maybe_const_set_copy = maybe_const_set;
                            f_ret_map_t& f_ret_map_copy = f_ret_map;
                            for(f_ptr_t feature : maybe_const_set) {
#endif

                                // get return-value
                                f_ret_t f_ret = feature->evaluate(basis_feature_maps[data_idx][outcome_idx]);

                                // compare
                                if(f_ret_map_copy[feature]!=f_ret) {
                                    DEBUG_OUT(4,"    different value (" << f_ret_map_copy[feature] << "/" << f_ret << "): " << *feature);
                                    non_const_set.insert(feature);
                                } else {
                                    DEBUG_OUT(4,"    same value (" << f_ret << "): " << *feature);
                                }
#ifdef USE_OMP
                            }
#else
                        }
#endif

                        // increment
                        ++outcome_idx;
                    }
                }

#ifdef USE_OMP
#pragma omp critical
#endif
                {
                    // erase non-const features from maybe-const set and return-value
                    // maps
                    for(f_ptr_t feature : non_const_set) {
                        DEBUG_OUT(3,"    non-const: " << *feature);
                        auto it_maybe = maybe_const_set.find(feature);
                        if(it_maybe!=maybe_const_set.end()) {
                            maybe_const_set.erase(it_maybe);
                        } else {
#ifndef USE_OMP
                            // in parallel evaluation two differend threads may
                            // try to remove the same feature so that it cannot
                            // be found by the second thread, this should not
                            // happen in non-parallel (same thing below)
                            DEBUG_DEAD_LINE;
#endif
                        }
                        auto it = f_ret_map.find(feature);
                        if(it!=f_ret_map.end()) {
                            f_ret_map.erase(it);
                        } else {
#ifndef USE_OMP
                            // see above for explanation
                            DEBUG_DEAD_LINE;
#endif
                        }
                    }

                    // print progress
                    if(DEBUG_LEVEL>0) {
                        ProgressBar::msg() << " (" << feature_set.size()-maybe_const_set.size() << ")";
                        ProgressBar::print(progress_idx,number_of_data_points);
                    }
                } // critical


#ifndef USE_OMP
                // increment
                ++data_idx;
            }
        }
#else
        ++progress_idx;
    }
#endif

    // feature that still may be const actually ARE const (for the given data)
    weight_map_t old_weights = get_weight_map();
    for(f_ptr_t const_feature : maybe_const_set) {
        DEBUG_OUT(3,"    const: " << *const_feature);
        auto it = feature_set.find(const_feature);
        if(it!=feature_set.end()) {
            feature_set.erase(it);
        } else {
            DEBUG_DEAD_LINE;
        }
    }
    apply_weight_map(old_weights);

    // terminate progress
    if(DEBUG_LEVEL>0) {
        ProgressBar::msg() << " (" << old_weights.size() << " --> " << feature_set.size() << ")";
        ProgressBar::terminate();
    }

    // feature set changed only if any features were actually erased
    if(old_weights.size()!=feature_set.size()) {
        feature_set_changed = true;
        return true;
    }

    return false;
}

bool TEM::update_basis_features() {

    if(DEBUG_LEVEL>0) {ProgressBar::init("Update basis feature set:  ");}

    f_ptr_set_t new_basis_features;
    int feature_n = feature_set.size();
    int feature_idx = 0;
    for(f_ptr_t feature : feature_set) {
        // get basis features
        auto and_feature = dynamic_pointer_cast<const AndFeature>(feature);
        if(and_feature!=nullptr) {
            auto subfeatures = and_feature->get_subfeatures();
            new_basis_features.insert(subfeatures.begin(),subfeatures.end());
        } else {
            DEBUG_DEAD_LINE;
        }
        // print progress
        if(DEBUG_LEVEL>0) {ProgressBar::print(feature_idx,feature_n);}
        // increment
        ++feature_idx;
    }

    // terminate progress
    if(DEBUG_LEVEL>0) {ProgressBar::terminate();}

    if(new_basis_features!=basis_features) {
        basis_features = new_basis_features;
        return true;
    } else {
        return false;
    }
}

void TEM::update_basis_feature_maps(bool recompute_all) {

    if(DEBUG_LEVEL>0) {ProgressBar::init("Update basis feature maps: ");}

    // get dimensions
    int data_n = number_of_data_points;
    int outcome_n = observation_space->space_size()*reward_space->space_size();

    // check if for matching dimensions
    if(basis_feature_maps.size()!=(uint)data_n || basis_feature_maps.front().size()!=(uint)outcome_n) {
        basis_feature_maps.assign(data_n,vector<basis_feature_map_t>(outcome_n));
        IF_DEBUG(1) {
            if(!recompute_all) {
                DEBUG_WARNING("Dimensions don't match, recomputation forced");
            }
        }
        recompute_all = true;
    }

    // remove unneeded and remember new basis feature for partial recomputation
    f_ptr_set_t new_basis_features;
    if(!recompute_all) {
        basis_feature_map_t& some_old_map = basis_feature_maps.front().front();
        // remove old ones
        {
            // find old
            f_ptr_set_t features_to_remove;
            for(f_ptr_t bf : some_old_map.get_list_of_features()) {
                if(basis_features.find(bf)==basis_features.end()) {
                    features_to_remove.insert(bf);
                }
            }
            // remove old
#ifdef USE_OMP
#pragma omp parallel for schedule(static) collapse(1)
#endif
            for(int data_idx = 0; data_idx<(int)number_of_data_points; ++data_idx) {
                for(int outcome_idx = 0; outcome_idx<outcome_n; ++outcome_idx) {
                    basis_feature_map_t& bf_map = basis_feature_maps[data_idx][outcome_idx];
                    for(f_ptr_t bf : features_to_remove) {
                        bf_map.erase_feature(bf);
                    }
                }
            }
        }
        // remember new ones
        {
            for(f_ptr_t bf : basis_features) {
                if(some_old_map.find(bf)==some_old_map.end()) {
                    new_basis_features.insert(bf);
                }
            }
        }
    }

    //-------------------------------------------------------------------//
    // cannot parallelize this because instance class is not thread-save //
    //-------------------------------------------------------------------//

    // for all data points
// #ifdef USE_OMP
// #pragma omp parallel for schedule(dynamic,1) collapse(1)
//     for(int data_idx = 0; data_idx<(int)number_of_data_points; ++data_idx) {
//         const_instance_ptr_t ins;
// #pragma omp critical
//         ins=instance_data.front()->const_first()->const_next(data_idx);
// #else
    int data_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {
// #endif

            // for all outcomes
            int outcome_idx = 0;
            for(observation_ptr_t obs : observation_space) {
                for(reward_ptr_t rew : reward_space) {

                    // get basis feature map for this data point and outcome
                    basis_feature_map_t& bf_map = basis_feature_maps[data_idx][outcome_idx];

                    //--------------------------//
                    // update basis feature map //
                    //--------------------------//
                    if(recompute_all) {
                        bf_map.clear();
                        for(f_ptr_t bf : basis_features) {
                            bf_map.insert_feature(bf,bf->evaluate(ins->const_prev(),ins->action,obs,rew));
                        }
                    } else {
                        for(f_ptr_t bf : new_basis_features) {
                            bf_map.insert_feature(bf,bf->evaluate(ins->const_prev(),ins->action,obs,rew));
                        }
                    }

                    // increment
                    ++outcome_idx;
                }
            }

// #ifdef USE_OMP
// #pragma omp critical
// #endif
//                 {
            // print progress
            if(DEBUG_LEVEL>0) {ProgressBar::print(data_idx,number_of_data_points);}
            // }

// #ifndef USE_OMP
            // increment
            ++data_idx;
        }
    }
// #else
//     }
// #endif

    // terminate progress
    if(DEBUG_LEVEL>0) {ProgressBar::terminate();}
}

void TEM::update_F_matrices() {

    int data_n = number_of_data_points;
    int feature_n = feature_set.size();
    int outcome_n = observation_space->space_size()*reward_space->space_size();

    if(DEBUG_LEVEL>0) {ProgressBar::init("Update F-matrices:         ");}

    #warning need to clear for correct dimensions -- why? --> write minimal example
    F_matrices.clear();
    F_matrices.resize(data_n,f_mat_t(feature_n,outcome_n));

    // for all data points
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic,1) collapse(1)
    for(int data_idx = 0; data_idx<(int)number_of_data_points; ++data_idx) {
#else
        int data_idx = 0;
        for(const_instance_ptr_t episode : instance_data) {
            for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {
#endif

                // get F-matrix for this data point
                f_mat_t& F_matrix = F_matrices[data_idx];

                // for all outcomes
                int outcome_idx = 0;
                for(observation_ptr_t obs : observation_space) {
                    for(reward_ptr_t rew : reward_space) {

                        // get basis feature map for this data point and outcome
                        basis_feature_map_t& bf_map = basis_feature_maps[data_idx][outcome_idx];

                        //-----------------//
                        // update F-matrix //
                        //-----------------//
                        int feature_idx = 0;
                        for(f_ptr_t feature : feature_set) {

                            // set entry to 1 for non-zero features
                            //if(feature->evaluate(ins->const_prev(),ins->action,obs,rew)!=0) { // directly evaluate feature
                            if(feature->evaluate(bf_map)!=0) {                                // evaluate feature via basis feature map
                                DEBUG_OUT(4,"(" << F_matrix.n_rows << "," << F_matrix.n_cols << ")/(" << feature_idx << "," << outcome_idx << ")");
                                F_matrix(feature_idx,outcome_idx) = 1;
                            }

                            // increment
                            ++feature_idx;
                        }

                        // increment
                        ++outcome_idx;
                    }
                }

                // print progress
#ifdef USE_OMP
#pragma omp critical
#endif
                {if(DEBUG_LEVEL>0) {ProgressBar::print(data_idx,number_of_data_points);}}

#ifndef USE_OMP
                // increment
                ++data_idx;
            }
        }
#else
    }
#endif

    // terminate progress
    if(DEBUG_LEVEL>0) {ProgressBar::terminate();}

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

void TEM::update_outcome_indices() {

    if(DEBUG_LEVEL>0) {ProgressBar::init("Update outcome indices:    ");}

    int data_n = number_of_data_points;

    outcome_indices.assign(data_n,-1);

    // for all data points
    int data_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {

            // for all outcomes
            int outcome_idx = 0;
            for(observation_ptr_t obs : observation_space) {
                for(reward_ptr_t rew : reward_space) {

                    //----------------------//
                    // update outcome index //
                    //----------------------//
                    if(obs==ins->observation && rew==ins->reward) {
                        outcome_indices[data_idx] = outcome_idx;
                    }

                    // increment
                    ++outcome_idx;
                }
            }

            // check if outcome index was set
            if(outcome_indices[data_idx]<0) {
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
}



double TEM::neg_log_likelihood(col_vec_t& grad, const col_vec_t& w) {

    /** Some issues with armadillo:
     *
     * -- simple exp(vector) prodoces 1x1 matrix, trunc_exp does not -- don't
     * know why */

#warning todo: write a minimal example for this behavior

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
    double obj = 0;
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
        const row_vec_t exp_lin = arma::trunc_exp(lin);
        const double z = sum(exp_lin);

        // debug output
        if(DEBUG_LEVEL>=3) {
            lin.print("lin:");
            exp_lin.print("exp_lin:");
        }

        // compute objective and gradient
        double obj_comp = lin(outcome_idx)-log(z);
        col_vec_t grad_comp = F.col(outcome_idx) - F*exp_lin.t()/z;

#ifdef USE_OMP
#pragma omp critical
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
    DEBUG_OUT(1,"Iteration " << iteration_nr << " (" << objective_evaluations << "), Likelihood = " << exp(-fx));
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
