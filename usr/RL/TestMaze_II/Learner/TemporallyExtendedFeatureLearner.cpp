#include "TemporallyExtendedFeatureLearner.h"

#include "ConjunctiveAdjacency.h"
#include "../util/util.h"
#include "../util/QtUtil.h"
#include "../util/ProgressBar.h"

#include <omp.h>
#define USE_OMP

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#include "../util/debug.h"

using util::Range;
using util::INVALID;

using std::vector;
using std::map;
using std::make_tuple;
using std::dynamic_pointer_cast;
using std::cout;
using std::endl;

using arma::zeros;

typedef TemporallyExtendedFeatureLearner TEFL;

TEFL::TemporallyExtendedFeatureLearner(std::shared_ptr<ConjunctiveAdjacency> N):
    N_plus(N)
{}

void TEFL::grow_feature_set() {
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

void TEFL::shrink_feature_set() {
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

TEFL::f_set_t TEFL::get_feature_set() {
    return feature_set;
}


void TEFL::set_feature_set(const f_set_t& new_set) {
    feature_set = new_set;
    weights.zeros(feature_set.size());
    feature_set_changed = true;
}

void TEFL::set_l1_factor(const double& l1) {
    l1_factor = l1;
}

void TEFL::print_features() const {
    cout << "Feature Set:" << endl;
    int f_idx = 0;
    for(f_ptr_t f : feature_set) {
        cout << QString("    %1: [%2]	")
            .arg(f_idx,4)
            .arg(weights(f_idx),7,'f',4) <<
            *f << endl;
        ++f_idx;
    }
}

void TEFL::print_training_data() const {
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

void TEFL::add_action_observation_reward_tripel(
    const action_ptr_t& action,
    const observation_ptr_t& observation,
    const reward_ptr_t& reward,
    const bool& new_episode
    ) {
    HistoryObserver::add_action_observation_reward_tripel(action,observation,reward,new_episode);
    data_changed = true;
}

double TEFL::optimize_weights_LBFGS() {

    DEBUG_OUT(2,"Optimize weights using L-BFGS");

    // return if no data available
    if(number_of_data_points==0) {
        DEBUG_WARNING("Cannot optimize weights without data");
        return 0;
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
    double obj_value = lbfgs.optimize(values);

    // transfer result / set weights
    weights = arma::conv_to<col_vec_t>::from(values);

    // print final message
    IF_DEBUG(1) {
        LBFGS_final_message(obj_value);
    }

    // return objective value
    return obj_value;
}

bool TEFL::check_derivatives(const int& number_of_samples,
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

void TEFL::clear_data() {
    HistoryObserver::clear_data();
    data_changed = true;
}

void TEFL::set_spaces(const action_ptr_t & a,
                      const observation_ptr_t & o,
                      const reward_ptr_t & r) {
    SpaceManager::set_spaces(a,o,r);
    update_outcome_n();
}

void TEFL::print_F_matrices(int n) {
    update();
    int data_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {
            // print instances
            {
                const_instance_ptr_t this_ins = ins;
                for(int i=0;i<=n; ++i) {
                    cout << "ins(t=" << i << "): " << this_ins << endl;
                    this_ins = this_ins->const_prev();
                }
            }
            // print features
            {
                int feature_idx = 0;
                for(f_ptr_t f : feature_set) {
                    cout << "    Feature: " << *f << endl;
                    int outcome_idx = 0;
                    if(outcome_type==OUTCOME_TYPE::ACTION) {
                        for(action_ptr_t act : action_space) {
                            cout << "        " << act << " --> " << F_matrices[data_idx](feature_idx,outcome_idx) << endl;
                            ++outcome_idx;
                        }
                    } else if(outcome_type==OUTCOME_TYPE::OBSERVATION_REWARD) {
                        for(observation_ptr_t obs : observation_space) {
                            for(reward_ptr_t rew : reward_space) {
                                cout << "        " << obs << "/" << rew << " --> " << F_matrices[data_idx](feature_idx,outcome_idx) << endl;
                                ++outcome_idx;
                            }
                        }
                    } else {
                        DEBUG_DEAD_LINE;
                    }
                    ++feature_idx;
                }
            }
            ++data_idx;
        }
    }
}

void TEFL::set_outcome_type(OUTCOME_TYPE t) {
    outcome_type = t;
    update_outcome_n();
}

void TEFL::update_outcome_n() {
    if(outcome_type==OUTCOME_TYPE::ACTION) {
        outcome_n = action_space->space_size();
    } else if(outcome_type==OUTCOME_TYPE::OBSERVATION_REWARD) {
        outcome_n = observation_space->space_size()*reward_space->space_size();
    } else {
        DEBUG_DEAD_LINE;
    }
}

TEFL::weight_map_t TEFL::get_weight_map() const {
    weight_map_t weight_map;
    int idx = 0;
    for(f_ptr_t f : feature_set) {
        weight_map[f] = weights(idx);
        ++idx;
    }
    return weight_map;
}

void TEFL::apply_weight_map(weight_map_t weight_map) {
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

bool TEFL::update() {

    DEBUG_OUT(3,"Check if everything is up to date");

    //---------------------------------------------//
    // some sanity checks for non-zero debug level //
    //---------------------------------------------//
    if(DEBUG_LEVEL>0) {
        // check size of weight vector
        if(weights.size()!=feature_set.size()) {
            DEBUG_ERROR("Weights and feature set have different size (" << weights.size() << "/" << feature_set.size() << ")");
            weights.zeros(feature_set.size());
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
        // check if spaces are set
        if(action_space==action_ptr_t() ||
           observation_space==observation_ptr_t() ||
           reward_space==reward_ptr_t()) {
            DEBUG_WARNING("spaces are not correctly initialized");
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
        return true; // update was performed
    }
    return false; // no update was performed
}

bool TEFL::update_basis_features() {

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
    if(DEBUG_LEVEL>0) {
        ProgressBar::msg() << " (" << basis_features.size() << " --> " << new_basis_features.size() << ")";
        ProgressBar::terminate();
    }

    if(new_basis_features!=basis_features) {
        basis_features = new_basis_features;
        return true;
    } else {
        return false;
    }
}

void TEFL::update_basis_feature_maps(bool recompute_all) {

    if(DEBUG_LEVEL>0) {ProgressBar::init("Update basis feature maps: ");}

    // get dimensions
    int data_n = number_of_data_points;

    // check for matching dimensions
    if(basis_features.size()==0) {
        IF_DEBUG(1) {
            ProgressBar::msg() << " (empty basis-feature set)";
            ProgressBar::terminate();
        }
        return;
    }
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
            if(outcome_type==OUTCOME_TYPE::ACTION) {
                for(action_ptr_t act : action_space) {
                    // get basis feature map for this data point and outcome
                    basis_feature_map_t& bf_map = basis_feature_maps[data_idx][outcome_idx];
                    //--------------------------//
                    // update basis feature map //
                    //--------------------------//
                    if(recompute_all) {
                        bf_map.clear();
                        for(f_ptr_t bf : basis_features) {
                            bf_map.insert_feature(bf,bf->evaluate(ins,act,ins->observation,ins->reward));
                        }
                    } else {
                        for(f_ptr_t bf : new_basis_features) {
                            bf_map.insert_feature(bf,bf->evaluate(ins,act,ins->observation,ins->reward));
                        }
                    }
                    // increment
                    ++outcome_idx;
                }
            } else if(outcome_type==OUTCOME_TYPE::OBSERVATION_REWARD) {
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
            } else {
                DEBUG_DEAD_LINE;
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

void TEFL::update_F_matrices() {

    int data_n = number_of_data_points;
    int feature_n = feature_set.size();

    if(DEBUG_LEVEL>0) {ProgressBar::init("Update F-matrices:         ");}

    F_matrices.assign(data_n,zeros<f_mat_t>(feature_n,outcome_n));

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
                if(outcome_type==OUTCOME_TYPE::ACTION) {
                    for(action_ptr_t act : action_space) {
                        // get basis feature map for this data point and outcome
                        basis_feature_map_t& bf_map = basis_feature_maps[data_idx][outcome_idx];
                        //-----------------//
                        // update F-matrix //
                        //-----------------//
                        int feature_idx = 0;
                        for(f_ptr_t feature : feature_set) {
                            // set entry to 1 for non-zero features
                            //if(feature->evaluate(ins->const_prev(),act,ins->observation,ins->reward)!=0) { // directly evaluate feature
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
                } else if(outcome_type==OUTCOME_TYPE::OBSERVATION_REWARD) {
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
                } else {
                    DEBUG_DEAD_LINE;
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

void TEFL::update_outcome_indices() {

    if(DEBUG_LEVEL>0) {ProgressBar::init("Update outcome indices:    ");}

    int data_n = number_of_data_points;

    outcome_indices.assign(data_n,-1);

    // for all data points
    int data_idx = 0;
    for(const_instance_ptr_t episode : instance_data) {
        for(const_instance_ptr_t ins=episode->const_first(); ins!=INVALID; ++ins) {

            // for all outcomes
            int outcome_idx = 0;
            if(outcome_type==OUTCOME_TYPE::ACTION) {
                for(action_ptr_t act : action_space) {
                    //----------------------//
                    // update outcome index //
                    //----------------------//
                    if(act==ins->action) {
                        outcome_indices[data_idx] = outcome_idx;
                    }
                    // increment
                    ++outcome_idx;
                }
            } else if(outcome_type==OUTCOME_TYPE::OBSERVATION_REWARD) {
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
            } else {
                DEBUG_DEAD_LINE;
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

bool TEFL::pick_non_const_features() {

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
        for(int outcome_idx : Range(outcome_n)) {
            // for all features that may be const
            for(f_ptr_t feature : maybe_const_set) {
                // initialize on first data point
                f_ret_map[feature] = feature->evaluate(basis_feature_maps[data_idx][outcome_idx]);
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
                for(int outcome_idx : Range(outcome_n)) {
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

LBFGS_Object::objective_t TEFL::get_LBFGS_objective() {
    return std::bind(&TEFL::LBFGS_objective, this, std::placeholders::_1, std::placeholders::_2);
}

LBFGS_Object::progress_t TEFL::get_LBFGS_progress() const {
    return std::bind(&TEFL::LBFGS_progress, this,
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
