#include "LinearQ.h"
#include "util.h"

#include <queue>
#include <set>
#include <list>
#include <utility> // for std::pair
#include <tuple>
#include <float.h> // for DBL_MAX

#include "lbfgs_codes.h"

#include "util/ProgressBar.h"

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#define DEBUG_STRING "LinearQ: "
#include "debug.h"

// maze_x_size
// maze_y_size
// k
USE_CONFIG_CONSTS;

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
        discount(d),
        lambda(nullptr),
        loss_terms_up_to_date(false)
{
    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-(int)k; --k_idx) {
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

    // also add a unit feature
    ConstFeature * const_feature = ConstFeature::create(1);
    basis_features.push_back(const_feature);
    DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
}

LinearQ::~LinearQ() {}

void LinearQ::add_action_state_reward_tripel(
        const action_t& action,
        const state_t& state,
        const reward_t& reward
) {
    // call function of parent class
    HistoryObserver::add_action_state_reward_tripel(action,state,reward);

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

    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.orthantwise_c = reg;
    param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
    if(max_iter>0) {
        param.max_iterations = max_iter;
    }

    // todo what values
    param.delta = 1e-3;   // change of objective (f-f')/f (default 0)
    param.epsilon = 1e-3; // change of parameters ||g||/max(1,||x||) (default 1e-5)

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    lambda = lbfgs_malloc(active_features.size()); // allocate lambda
    int ret = lbfgs(active_features.size(), lambda, &fx, static_evaluate_model, static_progress_model, this, &param);

    // Transfer weights and report result
    DEBUG_OUT(1, "L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");
    DEBUG_OUT(1,"loss = " << fx );
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        DEBUG_OUT(1, "    " <<
                active_features[f_idx].identifier() <<
                " --> t[" <<
                f_idx << "] = " <<
                lambda[f_idx]);
        feature_weights[f_idx] = lambda[f_idx];
    }
    DEBUG_OUT(1, "L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");
    DEBUG_OUT(1,"Loss = " << fx << " (sqrt = " << sqrt(fx) << ")" );
    DEBUG_OUT(1,"");

    // free lambda
    lbfgs_free(lambda);

    // write out loss
    if(loss!=nullptr) {
        *loss = fx;
    }

    return ret;
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
    return util::draw_random(max_actions);
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

    // mark loss terms as out-of-date
    loss_terms_up_to_date = false;
}

void LinearQ::erase_zero_features() {

    DEBUG_OUT(1,"Erase zero features...");

    // find non-zero features
    idx_t feature_n = active_features.size();
    vector<bool> is_non_zero(feature_n,false);
    for(const_instanceIt_t insIt=instance_data->const_first(); insIt!=INVALID; ++insIt) {
        for(idx_t f_idx=0; f_idx<feature_n; ++f_idx) {
            if(is_non_zero[f_idx]==false && active_features[f_idx].evaluate(insIt)!=0) {
                is_non_zero[f_idx] = true;
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

lbfgsfloatval_t LinearQ::static_evaluate_model(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t /*step*/
) {
    return ((LinearQ*)instance)->evaluate_model(x,g,n);
}

lbfgsfloatval_t LinearQ::evaluate_model(
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

int LinearQ::static_progress_model(
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
    return ((LinearQ*)instance)->progress_model(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

int LinearQ::progress_model(
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
                " --> t[" <<
                f_idx << "] = " <<
                x[f_idx]);
    }
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", loss = " << fx << ")");
    DEBUG_OUT(1,"");

    return 0;
}

void LinearQ::check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation) {

    // initialize arrays
    lbfgsfloatval_t * x = lbfgs_malloc(active_features.size());
    lbfgsfloatval_t * dx = lbfgs_malloc(active_features.size());
    lbfgsfloatval_t * grad = lbfgs_malloc(active_features.size());
    lbfgsfloatval_t * grad_dummy = lbfgs_malloc(active_features.size());

    // remember deviations
    lbfgsfloatval_t relative_deviation = 0;

    DEBUG_OUT(0,"Checking first derivative (#samples="<<number_of_samples<<", range=+/-"<<range<<", max_var="<<max_variation<<", max_rel_dev="<<max_relative_deviation);
    for(int count=0; count<number_of_samples; ++count) {

        // set test point
        for(uint x_idx=0; x_idx<active_features.size(); ++x_idx) {
            x[x_idx] = range * (2*drand48()-1);
            dx[x_idx] = (2*drand48()-1)*max_variation;
        }

        lbfgsfloatval_t fx = evaluate_model(x,grad,active_features.size());
        DEBUG_OUT(1, "fx = " << fx );
        for(uint x_idx=0; x_idx<active_features.size(); ++x_idx) {

            // go in positive direction
            x[x_idx] += dx[x_idx]/2.;
            lbfgsfloatval_t fx_plus = evaluate_model(x,grad_dummy,active_features.size());
            // go in negative direction
            x[x_idx] -= dx[x_idx];
            lbfgsfloatval_t fx_minus = evaluate_model(x,grad_dummy,active_features.size());
            // reset x
            x[x_idx] += dx[x_idx]/2.;

            // numerical gradient
            lbfgsfloatval_t ngrad = (fx_plus-fx_minus)/dx[x_idx];

            // check for deviations
            lbfgsfloatval_t current_relative_deviation = fabs(ngrad-grad[x_idx])/fabs(grad[x_idx]);
            if(current_relative_deviation>relative_deviation) {
                relative_deviation=current_relative_deviation;
            }

            // print result
            DEBUG_OUT(1,
                      "    diff[" << x_idx << "] = " << grad[x_idx]-ngrad <<
                      ", grad["   << x_idx << "] = " << grad[x_idx] <<
                      ", ngrad["  << x_idx << "] = " << ngrad <<
                      ", x["      << x_idx << "] = " << x[x_idx] <<
                      ", dx["     << x_idx << "] = " << dx[x_idx] <<
                      ", rel_dev["     << x_idx << "] = " << current_relative_deviation
                );


        }
    }
    if(relative_deviation>max_relative_deviation) {
        DEBUG_OUT(0, "ERRORS in first derivative found: max relative deviation = " << relative_deviation << " (tolerance = " << max_relative_deviation << ")" );
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

void LinearQ::update_loss_terms() {
    //---------------//
    // get dimension //
    //---------------//
    idx_t feature_n = active_features.size();

    //-----------------------//
    // prepare for computing //
    //-----------------------//
    vector<f_ret_t> * f_ret_t0 = new vector<f_ret_t>(feature_n);
    vector<f_ret_t> * f_ret_t1 = new vector<f_ret_t>(feature_n);
    const_instanceIt_t ins_t0 = instance_data->const_first();
    const_instanceIt_t ins_t1 = ins_t0 + 1;
    const_instanceIt_t ins_t2 = ins_t0 + 2;
    idx_t f_idx = 0;
    for( auto f : active_features ) {
        (*f_ret_t0)[f_idx] = f.evaluate(ins_t0, ins_t1->action, state_t(), reward_t());
        ++f_idx;
    }

    //-----------------//
    // reset variables //
    //-----------------//
    c = 0;
    rho = zeros(feature_n);
    L = zeros(feature_n,feature_n);

    //----------------//
    // fill variables //
    //----------------//
    size_t data_idx = 0;
    size_t data_size = instance_data->const_it().length_to_first() - 2;
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Updating Loss Terms");
    }
    while(ins_t1!=INVALID) {

        if(DEBUG_LEVEL>=1) {
            ProgressBar::print(data_idx, data_size);
        }

        // count data
        ++data_idx;

        // precompute feature values
        f_idx = 0;
        for( auto f : active_features ) {
            (*f_ret_t1)[f_idx] = f.evaluate(ins_t1, ins_t2->action, state_t(), reward_t());
            ++f_idx;
        }

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
        ++ins_t2;
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
