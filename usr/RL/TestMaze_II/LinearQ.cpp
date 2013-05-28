#include "LinearQ.h"
#include "util.h"

#include <queue>
#include <set>
#include <list>
#include <utility> // for std::pair
#include <tuple>
#include <float.h> // for DBL_MAX

#include <armadillo>

#define DEBUG_STRING "LinearQ: "
#define DEBUG_LEVEL 4
#include "debug.h"

using std::vector;
using std::set;
using std::list;
using std::cout;
using std::endl;
using std::pair;
using std::make_pair;
using std::tuple;
using std::make_tuple;
using std::get;
using std::priority_queue;

using arma::mat;
using arma::vec;
using arma::randu;
using arma::mat44;
using arma::zeros;

using util::INVALID;

LinearQ::LinearQ(const double& d):
        k(Data::k),
        instance_data(nullptr),
        discount(d)
{

    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-k; --k_idx) {
        // actions
        for(action_t action : actionIt_t::all) {
            ActionFeature * action_feature = ActionFeature::create(action,k_idx);
            basis_features.push_back(action_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        if(k_idx<0) { // present state and reward are not known for predicting value
            // states
            for(state_t state : stateIt_t::all) {
                StateFeature * state_feature = StateFeature::create(state,k_idx);
                basis_features.push_back(state_feature);
                DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
            }
            // reward
            for(reward_t reward : rewardIt_t::all) {
                RewardFeature * reward_feature = RewardFeature::create(reward,k_idx);
                basis_features.push_back(reward_feature);
                DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
            }
        }
    }
}

LinearQ::~LinearQ() {
    delete instance_data;
}

void LinearQ::add_action_state_reward_tripel(
        const action_t& action,
        const state_t& state,
        const reward_t& reward
) {
    // create instance data or append to existing
    if(instance_data==nullptr) {
        instance_data = instance_t::create(action,state,reward);
    } else {
        instance_data = instance_data->append_instance(action,state,reward);
    }
    DEBUG_OUT(2, "added (action,state,reward) = (" << action << "," << state << "," << reward << ")" );
}

double LinearQ::optimize(const double& reg) {

    //---------------//
    // get dimension //
    //---------------//
    idx_t feature_n = active_features.size();

    //---------------------------//
    // declare variables to fill //
    //---------------------------//
    double c = 0;
    vec rho = zeros(feature_n);
    mat L = zeros(feature_n,feature_n);
    vec w;

    //-----------------------//
    // prepare for computing //
    //-----------------------//
    // vector<f_ret_t> * f_ret_t0 = new vector<f_ret_t>(feature_n);
    // vector<f_ret_t> * f_ret_t1 = new vector<f_ret_t>(feature_n);
    const_instanceIt_t ins_t0 = instance_data->const_first();
    const_instanceIt_t ins_t1 = ins_t0 + 1;
    const_instanceIt_t ins_t2 = ins_t0 + 2;
    // idx_t f_idx = 0;
    // for( auto f : active_features ) {
    //     (*f_ret_t0)[f_idx] = f.evaluate(ins_t0, ins_t1->action, state_t(), reward_t());
    //     ++f_idx;
    // }

    //----------------//
    // fill variables //
    //----------------//
    size_t data_idx = 0;
    size_t data_size = instance_data->const_it().length_to_first() - 2;
    while(ins_t1!=INVALID) {

        if(DEBUG_LEVEL>=1) {
            util::print_progress(data_idx, data_size, 40, "Building matrix: ");
        }

        // count data
        ++data_idx;

        // precompute feature values
        // f_idx = 0;
        // for( auto f : active_features ) {
        //     (*f_ret_t1)[f_idx] = f.evaluate(ins_t1, at2, state_t(), reward_t());
        //     ++f_idx;
        // }

        //--------------------//
        // increment elements //
        //--------------------//

        // constant
        c += ins_t0->reward;

        // iterate through rows
        for(int j=0; j<feature_n; ++j) {

            // double factor1 = discount * (*f_ret_t1)[j] - (*f_ret_t0)[j];
            double factor1 = discount * active_features[j].evaluate(ins_t1, ins_t2->action, state_t(), reward_t());
            factor1 -= active_features[j].evaluate(ins_t0, ins_t1->action, state_t(), reward_t());

            // increment linear term
            rho(j) += ins_t0->reward * factor1;

            // iterate through columns
            for(int k=0; k<feature_n; ++k) {

                // double factor2 = discount * (*f_ret_t1)[k] - (*f_ret_t0)[k];
                double factor2 = discount * active_features[k].evaluate(ins_t1, ins_t2->action, state_t(), reward_t());
                factor2 -= active_features[k].evaluate(ins_t0, ins_t1->action, state_t(), reward_t());

                L(j,k) += factor1 * factor2;
            }
        }

        // swap precomputed feature values
        // vector<f_ret_t> * tmp = f_ret_t0;
        // f_ret_t0 = f_ret_t1;
        // f_ret_t1 = tmp;

        // increment instance iterators
        ++ins_t0;
        ++ins_t1;
    }

    // terminate progress bar
    if(DEBUG_LEVEL>=1) {
        cout << endl;
    }

    // delete precomputed feature values
    // delete f_ret_t0;
    // delete f_ret_t1;

    // normalize over data size
    c /= data_idx;
    rho /= data_idx;
    L /= data_idx;

    // add regularization to L
    L.diag() += reg;

    // print
    if(DEBUG_LEVEL>=3) {
        DEBUG_OUT(0,"    c " << c);
        DEBUG_OUT(0,"    r-Vector");
        rho.print();
        DEBUG_OUT(0,"    L-Matrix");
        L.print();
    }

    // solve system
    bool solved = solve(w, L, -rho);
    if(!solved) {
        DEBUG_OUT(0,"Error: Could not solve equations");
        return 0;
    }

    // remember weights
    feature_weights.clear();
    for(int w_idx=0; w_idx<feature_n; ++w_idx) {
        feature_weights.push_back(w(w_idx));
        DEBUG_OUT(1,"    Feature " << active_features[w_idx] << " --> " << w(w_idx) );
    }

    // calculate loss
    double loss = arma::as_scalar(c + 2*rho.t()*w + w.t()*L*w);
    DEBUG_OUT(2,"    Loss = " << loss );

    return loss;
}

void LinearQ::clear_data() {
    delete instance_data;
    instance_data = nullptr;
}

action_t LinearQ::get_max_value_action(const instance_t * i) {
    return action_t(); // todo
}

void LinearQ::add_candidates(const int& n) {
    // construct candidats
    construct_candidate_features(n);

    // build new active set from currently active and candidates
    set<AndFeature> new_active_set;
    for( auto act : active_features ) {
        new_active_set.insert(act);
    }
    for( auto can : candidate_features ) {
        new_active_set.insert(can);
    }

    // replace active by new active
    active_features.clear();
    for( auto new_act : new_active_set ) {
        active_features.push_back(new_act);
    }
}

void LinearQ::erase_zero_features(const double& threshold) {
    // remember old data and clear
    auto old_active = active_features;
    auto old_weights = feature_weights;
    active_features.clear();
    feature_weights.clear();

    // iterate trough features to select non-zero weighted
    for(idx_t idx=0; idx<old_active.size(); ++idx) {
        if(fabs(old_weights[idx])>threshold) {
            active_features.push_back(old_active[idx]);
            feature_weights.push_back(old_weights[idx]);
        }
    }
}

void LinearQ::construct_candidate_features(const int& n) {

    DEBUG_OUT(1, "Constructing candidate features of distance " << n << "...");

    if(n<0) {
        DEBUG_OUT(0, "    Multiplicity must be non-negative");
        return;
    }
    if(n==0) {
        DEBUG_OUT(1, "    Multiplicity is zero, no features constructed");
        return;
    }

    // Sets of augmenting features (cominations of n basis features)
    // and candidate feature sets (currently active features combined
    // with one of the augmenting features).
    set<AndFeature> augmenting_feature_set;
    set<AndFeature> candidate_feature_set;

    // construct augmenting features
    idx_t basis_n = basis_features.size();
    if(basis_n==0) {
        DEBUG_OUT(0,"Error: Cannot construct features, no basis features available");
        return;
    }
    vector<int> aug_idx(n,0);
    bool idx_ok = true;
    while(idx_ok) {
        // construct new augmenting feature from basis features
        AndFeature and_feature;
        for(int idx_idx=0; idx_idx<n; ++idx_idx) {
            and_feature = AndFeature(and_feature, *basis_features[aug_idx[idx_idx]]);
        }
        // add augmenting feature
        augmenting_feature_set.insert(and_feature);
        DEBUG_OUT(4,"    Inserted potential augmenting feature: " << and_feature );
        // increment indices
        for(int idx_idx=0; idx_idx<n; ++idx_idx) {
            DEBUG_OUT(5,"    Idx " << idx_idx << ": " << aug_idx[idx_idx] );
            if( aug_idx[idx_idx] < (basis_n-1) ) {
                aug_idx[idx_idx] += 1;
                break;
            } else {
                DEBUG_OUT(5,"    reset, idx=" << idx_idx );
                aug_idx[idx_idx] = 0;
                if(idx_idx==(n-1)) {
                    DEBUG_OUT(5,"    STOP" );
                    idx_ok = false;
                }
            }
        }
    }
    DEBUG_OUT(3,"    Using " << augmenting_feature_set.size() << " augmenting features");

    // construct candidate features
    if(active_features.size()==0) {
        DEBUG_OUT(3,"    No active features, inserting augmenting features directly" );
        for( AndFeature aug_f : augmenting_feature_set ) {
            candidate_feature_set.insert(aug_f);
            DEBUG_OUT(3,"    Inserted potential candidate feature: " << aug_f );
        }
    } else {
        for( AndFeature aug_f : augmenting_feature_set ) {
            for( AndFeature act_f : active_features ) {
                AndFeature candidate_feature(aug_f,act_f);
                candidate_feature_set.insert(candidate_feature);
                DEBUG_OUT(3,"    Inserted potential candidate feature: " << candidate_feature );
            }
        }
    }

    // use unique candidate features
    candidate_features.clear();
    for( AndFeature can_f : candidate_feature_set ) {
        candidate_features.push_back(can_f);
        DEBUG_OUT(2,"    Added candidate feature: " << can_f );
    }

    DEBUG_OUT(1, "    Constructed " << candidate_features.size() << " features");

    DEBUG_OUT(1, "DONE");
}

LinearQ::probability_t LinearQ::prior_probability(const state_t&, const reward_t& r) const {
    if(r==reward_t::min_reward) {
        return 1./(state_t::state_n);
    } else {
        return 0;
    }
}
