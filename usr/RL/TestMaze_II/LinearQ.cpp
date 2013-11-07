#include "LinearQ.h"

#include <queue>
#include <set>
#include <list>
#include <utility> // for std::pair
#include <tuple>
#include <float.h> // for DBL_MAX

#include "lbfgs_codes.h"

#include "util/ProgressBar.h"
#include "util.h"

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#define DEBUG_STRING "LinearQ: "
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

using util::Range;

using arma::mat;
using arma::vec;
using arma::mat44;
using arma::zeros;

using util::INVALID;

LinearQ::LinearQ(const double& d):
        discount(d),
        loss_terms_up_to_date(false)
{
    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-(int)Config::k; --k_idx) {
        // actions
        if(true) {
            for(action_t action : actionIt_t::all) {
                ActionFeature * action_feature = ActionFeature::create(action,k_idx);
                basis_features.push_back(action_feature);
                DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
            }
        }
        if(k_idx<0) { // present state is not known for predicting value
            // states
            for(state_t state : stateIt_t::all) {
                StateFeature * state_feature = StateFeature::create(state,k_idx);
                basis_features.push_back(state_feature);
                DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
            }
        }
        if(false) { // no correlated rewards
            // reward
            for(reward_t reward : rewardIt_t::all) {
                RewardFeature * reward_feature = RewardFeature::create(reward,k_idx);
                basis_features.push_back(reward_feature);
                DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
            }
        }
    }

    // also add a unit feature
    ConstFeature * const_feature = ConstFeature::create(1);
    basis_features.push_back(const_feature);
    DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
}

LinearQ::~LinearQ() {}

void LinearQ::add_action_state_reward_tripel(
        const action_t& action,
        const state_t& state,
        const reward_t& reward,
        const bool& new_episode
) {
    // call function of parent class
    HistoryObserver::add_action_state_reward_tripel(action,state,reward,new_episode);

    // mark loss terms as out-of-date
    loss_terms_up_to_date = false;
}

double LinearQ::optimize_ridge(const double& reg) {

    // get dimension
    idx_t feature_n = active_features.size();

    // update terms if necessary
    if(!loss_terms_up_to_date) {
        update_loss_terms();
    }

    // weights
    vec w;

    // print
    DEBUG_OUT(1,"    c (mean reward) = " << c);
    if(DEBUG_LEVEL>=3) {
        DEBUG_OUT(0,"    r-Vector");
        rho.print();
        DEBUG_OUT(0,"    L-Matrix");
        L.print();
    }

    // add regularization to L and set sign for rho
    L.diag() += reg;
    rho *= -1;

    // solve system
    DEBUG_OUT(1,"    Solving system...");
    bool solved = solve(w, L, rho);
    if(!solved) {
        DEBUG_OUT(0,"Error: Could not solve equations");
        return 0;
    }

    // remove regularization from L and unset sign for rho
    L.diag() -= reg;
    rho *= -1;

    // remember weights
    feature_weights.clear();
    for(int w_idx=0; w_idx<feature_n; ++w_idx) {
        feature_weights.push_back(w(w_idx));
        DEBUG_OUT(1,"    Feature (" << w_idx << "): " <<
                  active_features[w_idx] << " --> " << w(w_idx)
            );
    }

    // calculate loss
    double loss = loss_function(w);
    DEBUG_OUT(1,"    Loss = " << loss << " (sqrt = " << sqrt(loss) << ")" );

    return loss;
}

int LinearQ::optimize_l1(const double& reg, const int& max_iter, double * loss) {

    set_l1_factor(reg);
    set_maximum_iterations(max_iter);
    set_lbfgs_delta(1e-3);
    set_lbfgs_epsilon(1e-3);
    set_number_of_variables(active_features.size());

    int return_code;
    lbfgsfloatval_t fx = optimize(&return_code);


    // Transfer weights and report result
    for(int f_idx : Range(active_features.size())) {
        DEBUG_OUT(1, "    " <<
                  active_features[f_idx].identifier() <<
                  " --> t[" << f_idx << "] = " <<
                  lbfgs_variables[f_idx]
            );
        feature_weights[f_idx] = lbfgs_variables[f_idx];
    }
    DEBUG_OUT(1,"Loss = " << fx << " (sqrt = " << sqrt(fx) << ")" );

    // write out loss
    if(loss!=nullptr) {
        *loss = fx;
    }

    return return_code;
}

void LinearQ::clear_data() {
    // call function of parent class
    HistoryObserver::clear_data();

    // mark loss terms as out-of-date
    loss_terms_up_to_date = false;
}

LinearQ::action_t LinearQ::get_max_value_action(const instance_t * i) {
    vector<action_t> max_actions;
    double max_value = -DBL_MAX;
    for( auto action : actionIt_t::all ) {
        double value = 0;
        for(idx_t f_idx=0; f_idx<(idx_t)active_features.size(); ++f_idx) {
            value += feature_weights[f_idx]*active_features[f_idx].evaluate(i, action, state_t(), reward_t() );
        }
        if(value>max_value) {
            max_value = value;
            max_actions.assign(1,action);
        } else if(value==max_value) {
            max_actions.push_back(action);
        }
    }
    return util::random_select(max_actions);
}

void LinearQ::add_candidates(const int& n) {

    // construct candidats
    construct_candidate_features(n);

    DEBUG_OUT(1,"Add candidate features...");

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
        DEBUG_OUT(1,"    Add " << new_act << " to active features");
        active_features.push_back(new_act);
    }

    // resize weights and set to zero
    feature_weights.assign(active_features.size(),0);

    // erase const zero features
    erase_zero_features();

    // mark loss terms as out-of-date
    loss_terms_up_to_date = false;
}

void LinearQ::erase_zero_features() {

    DEBUG_OUT(1,"Erase zero features...");

    // find non-zero features
    idx_t feature_n = active_features.size();
    vector<bool> is_non_zero(feature_n,false);
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=util::INVALID; ++insIt) {
            for(idx_t f_idx=0; f_idx<feature_n; ++f_idx) {
                if(is_non_zero[f_idx]==false && active_features[f_idx].evaluate(insIt)!=0) {
                    is_non_zero[f_idx] = true;
                }
            }
        }
    }

    // keep only non-zero
    auto old_active = active_features;
    auto old_weights = feature_weights;
    active_features.clear();
    feature_weights.clear();
    for(idx_t f_idx=0; f_idx<feature_n; ++f_idx) {
        if(is_non_zero[f_idx]) {
            active_features.push_back(old_active[f_idx]);
            feature_weights.push_back(old_weights[f_idx]);
            DEBUG_OUT(1,"    keep  " << old_active[f_idx] );
        } else {
            DEBUG_OUT(1,"    erase " << old_active[f_idx] );
        }
    }

    DEBUG_OUT(1,"DONE");

    // mark loss terms as out-of-date
    loss_terms_up_to_date = false;
}

void LinearQ::erase_zero_weighted_features(const double& threshold) {

    DEBUG_OUT(1,"Erase zero weighted features...");

    // remember old data and clear
    auto old_active = active_features;
    auto old_weights = feature_weights;
    active_features.clear();
    feature_weights.clear();

    // iterate trough features to select non-zero weighted
    for(idx_t f_idx=0; f_idx<(idx_t)old_active.size(); ++f_idx) {
        if(fabs(old_weights[f_idx])>threshold) {
            active_features.push_back(old_active[f_idx]);
            feature_weights.push_back(old_weights[f_idx]);
            DEBUG_OUT(1,"    keep  " << old_active[f_idx] );
        } else {
            DEBUG_OUT(1,"    erase " << old_active[f_idx] );
        }
    }

    DEBUG_OUT(1,"DONE");

    // mark loss terms as out-of-date
    loss_terms_up_to_date = false;
}

lbfgsfloatval_t LinearQ::objective(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {
    switch(optimization_type) {
    case OPTIMIZATION_TYPE::TD_RIDGE:
        // is optimized in closed form, not with L-BFGS
        DEBUG_DEAD_LINE;
        break;
    case OPTIMIZATION_TYPE::TD_L1:
        return td_error_objective(x,g,n);
        break;
    case OPTIMIZATION_TYPE::BELLMAN:
        // TODO
        return td_error_objective(x,g,n);
        break;
    default:
        DEBUG_DEAD_LINE;
    }
    return LBFGS_Optimizer::objective(x,g,n);
}

lbfgsfloatval_t LinearQ::td_error_objective(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {

    //------------------------------------//
    // Compute loss function and gradient //
    //------------------------------------//

    // x[f_idx] : current weights
    // fx       : loss function
    // g[f_idx] : gradient
    // n        : dimension (number of features)

    // update loss terms if necessary
    if(!loss_terms_up_to_date) {
        update_loss_terms();
    }

    // get current weights
    vec w(n);
    for(int i=0; i<n; i++) {
        w(i) = x[i];
    }

    // get loss for given weights
    lbfgsfloatval_t fx = loss_function(w);

    // get gradient for given weights
    vec dw = loss_gradient(w);

    // transfer gradient back
    for(int i=0; i<n; i++) {
        g[i] = dw(i);
    }

    return fx;
}

lbfgsfloatval_t LinearQ::bellman_objective(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {

    // reset objective and gradient
    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }

    // start progress bar
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Evaluating: ");
    }

    // iterate through data
    idx_t instance_idx = 0;
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt) {

            // print progress information
            if(DEBUG_LEVEL>0) {
                ProgressBar::print(instance_idx++, number_of_data_points-1);
            }

            // ignore last instance
            if(insIt+1==INVALID) {
                break;
            }
            action_t action_tp1 = (insIt+1)->action;
            reward_t reward_tp1 = (insIt+1)->reward;

            //-------------------------------//
            //          variables            //
            //-------------------------------//

            // f(i_t,a+1): features for time t
            vector<f_ret_t> f_it_atp1(n);

            // f(i_t+1,a): features for time t+1 for all actions a
            vector<vector<f_ret_t> > f_itp1_a(action_t::action_n,vector<double>(n));

            // Q(i_t,a_t+1): Q-function for time t
            double Q_it_atp1 = 0;

            // Q(i_t+1,a): Q-function for time t+1 for for all actions a
            vector<f_ret_t> Q_itp1_a(action_t::action_n,0);

            // sum_a exp[alpha Q(i_t+1,a)]
            double sumExp = 0;

            // sum_a exp[alpha Q(i_t+1,a_t)]
            double sumQExp = 0;

            //-------------------------------//
            // calculate the different terms //
            //-------------------------------//

            // evaluate feature and calculate Q-functions
            for(uint f_idx : Range(n)) { // iterate through features
                // evaluate feature
                f_ret_t f_ret = active_features[f_idx].evaluate(insIt,action_tp1,state_t(),reward_t());
                f_it_atp1[f_idx] = f_ret;
                // increment Q-function
                double f_weight = x[f_idx];
                Q_it_atp1 += f_weight*f_ret;
                idx_t action_idx = 0;
                for(action_t action : actionIt_t::all) {
                    // evaluate feature
                    f_ret_t f_ret_a = active_features[f_idx].evaluate(insIt,action,state_t(),reward_t());
                    f_itp1_a[action_idx][f_idx] = f_ret_a;
                    // increment Q-function
                    Q_itp1_a[action_idx] += f_weight*f_ret_a;
                    // increment action idx
                    ++action_idx;
                }
            }

            // calculate sums
            idx_t action_idx = 0;
            for(action_t action : actionIt_t::all) {
                double expQ = exp(alpha*Q_itp1_a[action_idx]);
                sumExp += expQ;
                sumQExp += Q_itp1_a[action_idx]*expQ;
                ++action_idx;
            }

            //-------------------------------//
            // update objective and gradient //
            //-------------------------------//

            // objective
            double bellman_error = reward_tp1 + discount*(sumQExp/sumExp) - Q_it_atp1;
            fx += bellman_error*bellman_error;

            // gradient
            for(uint f_idx : Range(n)) {
                double aSum = 0;
                idx_t action_idx = 0;
                for(action_t action : actionIt_t::all) {
                    double inner_terms = 1 + alpha*Q_itp1_a[action_idx] - alpha*(sumQExp/sumExp);
                    aSum += (exp(alpha*Q_itp1_a[action_idx])/sumExp)*inner_terms*f_itp1_a[action_idx][f_idx];
                    ++action_idx;
                }
                g[f_idx] += 2*bellman_error*(f_it_atp1[f_idx] - discount*aSum);
            }
        }
    }

    // terminate progress bar
    if(DEBUG_LEVEL>0) {
        ProgressBar::terminate();
    }

    return fx;
}

int LinearQ::progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t * /*g*/,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t /*gnorm*/,
        const lbfgsfloatval_t /*step*/,
        int /*n*/,
        int k,
        int /*ls*/
) {
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", loss = " << fx << "):");
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        DEBUG_OUT(1, "    " <<
                active_features[f_idx].identifier() <<
                " --> t[" << f_idx << "] = " <<
                  x[f_idx]);
    }
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", loss = " << fx << ")");
    DEBUG_OUT(1,"");

    return 0;
}

lbfgsfloatval_t LinearQ::optimize(
    int * return_code,
    std::string * return_code_description
    ) {

    // set correct dimensionality
    set_number_of_variables(active_features.size());

    // perform optimization
    lbfgsfloatval_t fx = LBFGS_Optimizer::optimize(return_code, return_code_description);

    // transfer weights and report result
    for(int f_idx : Range(active_features.size())) {
        DEBUG_OUT(1, "    " <<
                  active_features[f_idx].identifier() <<
                  " --> t[" << f_idx << "] = " <<
                  lbfgs_variables[f_idx]
            );
        feature_weights[f_idx] = lbfgs_variables[f_idx];
    }
    DEBUG_OUT(1,"Loss = " << fx << " (sqrt = " << sqrt(fx) << ")" );

    // return loss
    return fx;
}

LinearQ& LinearQ::set_l1_factor(lbfgsfloatval_t f ) {
    LBFGS_Optimizer::set_l1_factor(f);
    return *this;
}

LinearQ& LinearQ::set_lbfgs_delta(lbfgsfloatval_t f ) {
    LBFGS_Optimizer::set_lbfgs_delta(f);
    return *this;
}

LinearQ& LinearQ::set_maximum_iterations(unsigned int n ) {
    LBFGS_Optimizer::set_maximum_iterations(n);
    return *this;
}

LinearQ& LinearQ::set_lbfgs_epsilon(lbfgsfloatval_t f ) {
    LBFGS_Optimizer::set_lbfgs_epsilon(f);
    return *this;
}

LinearQ& LinearQ::set_optimization_type(OPTIMIZATION_TYPE t ) {
    optimization_type = t;
    return *this;
}

void LinearQ::update_loss_terms() {
    //---------------//
    // get dimension //
    //---------------//
    idx_t feature_n = active_features.size();

    //-----------------//
    // reset variables //
    //-----------------//
    c = 0;
    rho = zeros(feature_n);
    L = zeros(feature_n,feature_n);

    //-----------------------//
    // prepare for computing //
    //-----------------------//
    size_t data_idx = 0;
    vector<f_ret_t> * f_ret_t0 = new vector<f_ret_t>(feature_n);
    vector<f_ret_t> * f_ret_t1 = new vector<f_ret_t>(feature_n);

    //----------------//
    // fill variables //
    //----------------//
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Updating Loss Terms: ");
    }
    for(instance_t * current_episode : instance_data) {

        // iterators with delay of one
        const_instanceIt_t ins_t0 = current_episode->const_first();
        const_instanceIt_t ins_t1 = ins_t0 + 1;

        // precompute feature values
        idx_t f_idx = 0;
        for( auto f : active_features ) {
            (*f_ret_t0)[f_idx] = f.evaluate(ins_t0);
            ++f_idx;
        }

        // iterate through current episode
        while(ins_t1!=INVALID) {

            // print progress
            if(DEBUG_LEVEL>=1) {
                ProgressBar::print(data_idx, number_of_data_points);
            }

            // precompute feature values
            idx_t f_idx = 0;
            for( auto f : active_features ) {
                (*f_ret_t1)[f_idx] = f.evaluate(ins_t1);
                ++f_idx;
            }

            // count data
            ++data_idx;

            //--------------------//
            // increment elements //
            //--------------------//

            // constant
            c += pow(ins_t0->reward,2);

            // iterate through rows
            for(int j=0; j<feature_n; ++j) {

                double factor1 = discount * (*f_ret_t1)[j] - (*f_ret_t0)[j];

                // increment linear term
                rho(j) += ins_t0->reward * factor1;

                // iterate through columns
                for(int k=0; k<feature_n; ++k) {

                    double factor2 = discount * (*f_ret_t1)[k] - (*f_ret_t0)[k];

                    L(j,k) += factor1 * factor2;
                }
            }

            // swap precomputed feature values (f_ret_t0
            // does not need to be recomputed )
            vector<f_ret_t> * tmp = f_ret_t0;
            f_ret_t0 = f_ret_t1;
            f_ret_t1 = tmp;

            // increment instance iterators
            ++ins_t0;
            ++ins_t1;
        }
    }

    // terminate progress bar
    if(DEBUG_LEVEL>=1) {
        ProgressBar::terminate();
    }

    // delete precomputed feature values
    delete f_ret_t0;
    delete f_ret_t1;

    // normalize over data size
    c /= data_idx;
    rho /= data_idx;
    L /= data_idx;

    // set up-to-date flag
    loss_terms_up_to_date = true;
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
