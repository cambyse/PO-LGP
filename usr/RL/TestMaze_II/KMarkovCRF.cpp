#include "KMarkovCRF.h"
#include "lbfgs_codes.h"
#include "util.h"
#include "util/ProgressBar.h"

#include "QtUtil.h" // for << operator

#include <list>
#include <map>

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#define DEBUG_STRING "CRF: "
#include "debug.h"

using std::vector;
using std::list;
using std::tuple;
using std::make_tuple;
using std::get;
using std::pair;
using std::make_pair;
using std::set;
using std::cout;
using std::endl;
using std::map;

using util::INVALID;

//#define USE_RELATIVE_FEATURES

const KMarkovCRF::PRECOMPUTATION_TYPE KMarkovCRF::precomputation_type = KMarkovCRF::BASE_LOOK_UP;

KMarkovCRF::KMarkovCRF():
        lambda(nullptr),
        lambda_candidates(nullptr),
        old_active_features_size(0),
        feature_values_precomputed(false),
        use_stochastic_sparsification(false),
        exclude_data_1(0),
        exclude_data_2(0)
{

    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-(int)Config::k; --k_idx) {
        // actions
        for(actionIt_t action=actionIt_t::first(); action!=INVALID; ++action) {
            ActionFeature * action_feature = ActionFeature::create(action,k_idx);
            basis_features.push_back(action_feature);
            DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // states
        for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
            StateFeature * state_feature = StateFeature::create(state,k_idx);
            basis_features.push_back(state_feature);
            DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // reward
        if(k_idx==0) { // take only the current reward into account (for predicting only)
            for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID; ++reward) {
                RewardFeature * reward_feature = RewardFeature::create(reward,k_idx);
                basis_features.push_back(reward_feature);
                DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
            }
        }
    }

#ifdef USE_RELATIVE_FEATURES
    // relative state features
    RelativeStateFeature * relative_state_feature;
    relative_state_feature = RelativeStateFeature::create(1,0,-1,0);
    basis_features.push_back(relative_state_feature);
    DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
    relative_state_feature = RelativeStateFeature::create(0,1,-1,0);
    basis_features.push_back(relative_state_feature);
    DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
    relative_state_feature = RelativeStateFeature::create(-1,0,-1,0);
    basis_features.push_back(relative_state_feature);
    DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
    relative_state_feature = RelativeStateFeature::create(0,-1,-1,0);
    basis_features.push_back(relative_state_feature);
    DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
    relative_state_feature = RelativeStateFeature::create(0,0,-1,0);
    basis_features.push_back(relative_state_feature);
    DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
#endif
}

KMarkovCRF::~KMarkovCRF() {
    lbfgs_free(lambda);
    lbfgs_free(lambda_candidates);
}

lbfgsfloatval_t KMarkovCRF::static_evaluate_model(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t /*step*/
) {
    return ((KMarkovCRF*)instance)->evaluate_model(x,g,n);
}

lbfgsfloatval_t KMarkovCRF::evaluate_model(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {

    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }

    // Print parameter vector //
    if(DEBUG_LEVEL>=2) {
        DEBUG_OUT(2, "Parameter vector:");
        for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
            DEBUG_OUT(2, "    t[" << f_idx << "] = " << x[f_idx] );
        }
    }

    //--------------------------------------//
    // Compute data likelihood and gradient //
    //--------------------------------------//

    // precompute feature value if not already done
    switch(precomputation_type) {
    case NONE:
        // Nothing to do
        break;
    case COMPOUND_LOOK_UP:
        if(!feature_values_precomputed) {
            precompute_compound_feature_values();
            feature_values_precomputed = true;
        }
        break;
    case BASE_LOOK_UP:
        if(!feature_values_precomputed) {
            precompute_base_feature_values();
            feature_values_precomputed = true;
        }
        break;
    default:
        DEBUG_DEAD_LINE;
    }

    idx_t feature_n = active_features.size();
    if(feature_n!=n) {
        DEBUG_OUT(0,"Error: number of features is different from number of parameters but no parameter binding allowed");
    }

    // iterate through data
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Evaluating: ");
    }
    double sumFNN;                     // sumF(x(n),y(n))
    double sumExpN;                    // normalization Z(x)
    vector<double> sumFExpNF(n,0.0);   // sumFExp(x(n),F)
    idx_t instance_idx = 0;
    ignored_data = 0;
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt, ++instance_idx) {

            // last action is not changed while iterating through states and rewards
            action_t action = insIt->action;

            // exclude data
            double data_percent = double(instance_idx+1)/number_of_data_points;
            if(data_percent>exclude_data_1 && data_percent<exclude_data_2) {
                continue;
            }

            // sparsify data
            if(use_stochastic_sparsification) {
                if(pow(data_probabilities[instance_idx],sparse_beta)>drand48()) {
                    data_probabilities[instance_idx] /= sparse_alpha;
                    ++ignored_data;
                    continue;
                }
            }

            // print progress information
            if(DEBUG_LEVEL>0) {
                ProgressBar::print(instance_idx, number_of_data_points);
            }

            // store evaluations for this instance in array (for speed up)
            vector<f_ret_t> instance_evaluations(feature_n,0);
            vector<vector<f_ret_t> > state_reward_evaluations(feature_n,vector<f_ret_t>(state_t::state_n*reward_t::reward_n,0));

            // reset sums
            sumFNN = 0;
            sumExpN = 0;
            sumFExpNF.assign(n,0.0);

            //-------------------------------//
            // calculate the different terms //
            //-------------------------------//

            // calculate sumF(x(n),y(n))
            for(uint f_idx=0; f_idx<feature_n; ++f_idx) { // sum over features
                // compute and store
                f_ret_t f_ret;
                switch(precomputation_type) {
                case NONE:
                    f_ret = active_features[f_idx].evaluate(insIt-1, insIt->action, insIt->state, insIt->reward);
                    break;
                case COMPOUND_LOOK_UP:
                {
                    idx_t pre_idx = precomputed_feature_idx(instance_idx,f_idx,feature_n);
                    f_ret = compound_feature_values[pre_idx];
                    break;
                }
                case BASE_LOOK_UP:
                    f_ret = active_features[f_idx].evaluate(base_feature_values[instance_idx][base_feature_indices[instance_idx]]);
                    break;
                default:
                    DEBUG_DEAD_LINE;
                }
                instance_evaluations[f_idx] = f_ret;
                sumFNN += x[f_idx]*f_ret;
            }

            // calculate sumExp(x(n))
            idx_t state_reward_idx=0;
            for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
                for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID; ++reward) {

                    // calculate sumF(x(n),y')
                    double sumFN = 0;
                    for(uint f_idx=0; f_idx<feature_n; ++f_idx) { // sum over features
                        // compute and store
                        f_ret_t f_ret;
                        switch(precomputation_type) {
                        case NONE:
                            f_ret = active_features[f_idx].evaluate(insIt-1,action,state,reward);
                            break;
                        case COMPOUND_LOOK_UP:
                        {
                            idx_t pre_idx = precomputed_feature_idx(instance_idx,f_idx,feature_n,state,reward);
                            f_ret = compound_feature_values[pre_idx];
                            break;
                        }
                        case BASE_LOOK_UP:
                            f_ret = active_features[f_idx].evaluate(base_feature_values[instance_idx][state_reward_idx]);
                            break;
                        default:
                            DEBUG_DEAD_LINE;
                        }
                        state_reward_evaluations[f_idx][state_reward_idx] = f_ret;
                        sumFN += x[f_idx]*f_ret;
                    }

                    // increment sumExp(x(n))
                    sumExpN += exp( sumFN );

                    // increment sumFExp(x(n),F)
                    for(int lambda_idx=0; lambda_idx<n; ++lambda_idx) { // for all parameters/gradient components
                        // in case of parameter binding additionally sum over all features belonging to this parameter
                        sumFExpNF[lambda_idx] += state_reward_evaluations[lambda_idx][state_reward_idx] * exp( sumFN );
                    }

                    // increment state-reward index
                    ++state_reward_idx;
                }
            }

            //----------------------------------//
            // increment objective and gradient //
            //----------------------------------//

            // log-probability for this datum
            double log_prob = sumFNN - log( sumExpN );

            // weight for this datum
            double weight = 1;

            // update probability for data sparsification
            if(use_stochastic_sparsification) {
                weight = 1./(1-pow(data_probabilities[instance_idx],sparse_beta));
                data_probabilities[instance_idx] = exp(log_prob);
            }

            // increment fx (objective)
            fx += weight * log_prob;

            // increment gradient
            for(int lambda_idx=0; lambda_idx<n; ++lambda_idx) { // for all parameters/gradient components
                g[lambda_idx] -= weight * sumFExpNF[lambda_idx]/sumExpN;

                // in case of parameter binding additionally sum over all features belonging to this parameter
                g[lambda_idx] += weight * instance_evaluations[lambda_idx];
            }
        }
    }

    // terminate progress bar
    if(DEBUG_LEVEL>0) {
        ProgressBar::terminate();
        DEBUG_OUT(1,QString("    Ignored %1 out of %2 data points (%3% to %4% excluded)")
                  .arg(ignored_data)
                  .arg(number_of_data_points)
                  .arg((int)(100*exclude_data_1))
                  .arg((int)(100*exclude_data_2))
            );
    }

    // use NEGATIVE log likelihood (blfgs minimizes the objective)
    // use mean value per data point
    fx *= -1./number_of_data_points;
    for(int i=0; i<n; i++) {
        g[i] *= -1./number_of_data_points;
    }

    return fx;
}

int KMarkovCRF::static_progress_model(
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
    return ((KMarkovCRF*)instance)->progress_model(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

int KMarkovCRF::progress_model(
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
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", p = " << exp(-fx) << "):");
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        DEBUG_OUT(1, "    " <<
                active_features[f_idx].identifier() <<
                " --> t[" <<
                f_idx << "] = " <<
                x[f_idx]);
    }
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", p = " << exp(-fx) << "):");
    DEBUG_OUT(1,QString("    Ignored %1 out of %2 data points (%3% to %4% excluded)")
              .arg(ignored_data)
              .arg(number_of_data_points)
              .arg((int)(100*exclude_data_1))
              .arg((int)(100*exclude_data_2))
        );
    DEBUG_OUT(1,"");

    return 0;
}

int KMarkovCRF::optimize_model(lbfgsfloatval_t l1,
                               unsigned int max_iter,
                               lbfgsfloatval_t * mean_likelihood,
                               bool stochastic_sparsification,
                               double alpha,
                               double beta) {

    // Check size of parameter vector //
    check_lambda_size(lambda,active_features,old_active_features_size);

    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.orthantwise_c = l1;
    param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
    if(max_iter>0) {
        param.max_iterations = max_iter;
    }

    // todo what values
    param.delta = 1e-5;   // change of objective (f-f')/f (default 0)
    param.epsilon = 1e-5; // change of parameters ||g||/max(1,||x||) (default 1e-5)

    // stochastic sparsification
    if(stochastic_sparsification) {
        use_stochastic_sparsification = true;
        data_probabilities.assign(number_of_data_points,0);
        sparse_alpha = alpha;
        sparse_beta = beta;
    } else {
        use_stochastic_sparsification = false;
    }

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(active_features.size(), lambda, &fx, static_evaluate_model, static_progress_model, this, &param);

    // Report the result.
    DEBUG_OUT(1, "L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");
    DEBUG_OUT(1,"mean likelihood = " << exp(-fx) );
    // for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
    //     DEBUG_OUT(1, "    " <<
    //             active_features[f_idx].identifier() <<
    //             " --> t[" <<
    //             f_idx << "] = " <<
    //             lambda[f_idx]);
    // }
    // DEBUG_OUT(1, "L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");
    // DEBUG_OUT(1,"mean likelihood = " << exp(-fx) );
    DEBUG_OUT(1,"");

    if(mean_likelihood!=nullptr) {
        *mean_likelihood = exp(-fx);
    }

    return ret;
}

lbfgsfloatval_t KMarkovCRF::static_evaluate_candidates(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t /*step*/
) {
    return ((KMarkovCRF*)instance)->evaluate_candidates(x,g,n);
}

lbfgsfloatval_t KMarkovCRF::evaluate_candidates(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
) {

    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }

    // Print parameter vector //
    if(DEBUG_LEVEL>=2) {
        DEBUG_OUT(2, "Parameter vector:");
        for(uint f_idx=0; f_idx<candidate_features.size(); ++f_idx) {
            DEBUG_OUT(2, "    t[" << f_idx << "] = " << x[f_idx] );
        }
    }

    //--------------------------------------//
    // Compute data likelihood and gradient //
    //--------------------------------------//

    // precompute feature value if not already done
    switch(precomputation_type) {
    case NONE:
        // Nothing to do
        break;
    case COMPOUND_LOOK_UP:
        if(!feature_values_precomputed) {
            precompute_compound_feature_values();
            feature_values_precomputed = true;
        }
        break;
    case BASE_LOOK_UP:
        if(!feature_values_precomputed) {
            precompute_base_feature_values();
            feature_values_precomputed = true;
        }
        break;
    default:
        DEBUG_DEAD_LINE;
    }

    // number of features
    idx_t active_n = active_features.size();
    idx_t candidate_n = candidate_features.size();
    if(candidate_n!=n) {
        DEBUG_OUT(0,"Error: number of candidate features is different from number of parameters but no parameter binding allowed");
    }

    // reset scores
    candidate_feature_scores.assign(candidate_n,0);

    // iterate through data
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Evaluating: ");
    }
    vector<double> candidate_sumFNN;    // sumF(x(n),y(n)) for each candidate
    vector<double> candidate_sumExpN;   // normalization Z(x) for each candidate
    vector<double> candidate_sumFExpNF; // sumFExp(x(n),F) for each candidate
    idx_t instance_idx = 0;
    ignored_data = 0;
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt, ++instance_idx) {

            // last action is not changed while iterating through states and rewards
            action_t action = insIt->action;

            // print progress information
            if(DEBUG_LEVEL>0) {
                ProgressBar::print(instance_idx, number_of_data_points);
            }

            // store evaluations for this instance in array (for speed up)
            /* vector<f_ret_t> active_instance_evaluations(active_n,0); */
            vector<f_ret_t> candidate_instance_evaluations(candidate_n,0);
            /* vector<vector<f_ret_t> > active_state_reward_evaluations(active_n,vector<f_ret_t>(state_t::state_n*reward_t::reward_n,0)); */
            /* vector<vector<f_ret_t> > candidate_state_reward_evaluations(candidate_n,vector<f_ret_t>(state_t::state_n*reward_t::reward_n,0)); */

            //-------------------------------//
            // calculate the different terms //
            //-------------------------------//

            // calculate sumF(x(n),y(n))
            double active_sumFNN = 0;
            for(uint f_idx=0; f_idx<active_n; ++f_idx) { // sum over active features
                // compute and store
                f_ret_t f_ret;
                switch(precomputation_type) {
                case NONE:
                    f_ret = active_features[f_idx].evaluate(insIt-1, insIt->action, insIt->state, insIt->reward);
                    break;
                case COMPOUND_LOOK_UP:
                {
                    idx_t pre_idx = precomputed_feature_idx(instance_idx,f_idx,active_n);
                    f_ret = compound_feature_values[pre_idx];
                    break;
                }
                case BASE_LOOK_UP:
                    f_ret = active_features[f_idx].evaluate(base_feature_values[instance_idx][base_feature_indices[instance_idx]]);
                    break;
                default:
                    DEBUG_DEAD_LINE;
                }
                /* active_instance_evaluations[f_idx] = f_ret; */
                active_sumFNN += lambda[f_idx]*f_ret;
            }
            candidate_sumFNN.assign(n,active_sumFNN); // initialize with commond terms in sum
            for(uint f_idx=0; f_idx<candidate_n; ++f_idx) { // sum over candidate features
                // compute and store
                f_ret_t f_ret;
                switch(precomputation_type) {
                case NONE:
                case COMPOUND_LOOK_UP:
                    f_ret = candidate_features[f_idx].evaluate(insIt-1, insIt->action, insIt->state, insIt->reward);
                    break;
                case BASE_LOOK_UP:
                    f_ret = candidate_features[f_idx].evaluate(base_feature_values[instance_idx][base_feature_indices[instance_idx]]);
                    break;
                default:
                    DEBUG_DEAD_LINE;
                }
                candidate_instance_evaluations[f_idx] = f_ret;
                candidate_sumFNN[f_idx] += x[f_idx]*f_ret; // add individual terms for candidates
            }

            // calculate sumExp(x(n)) and sumFExp(x(n),F)
            candidate_sumExpN.assign(n,0.0);
            candidate_sumFExpNF.assign(n,0.0);
            idx_t state_reward_idx=0;
            for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
                for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID; ++reward) {

                    // calculate sumF(x(n),y')
                    double active_sumFN = 0; // common terms for all candidate features
                    for(uint f_idx=0; f_idx<active_n; ++f_idx) { // sum over features
                        // compute and store
                        f_ret_t f_ret;
                        switch(precomputation_type) {
                        case NONE:
                            f_ret = active_features[f_idx].evaluate(insIt-1,action,state,reward);
                            break;
                        case COMPOUND_LOOK_UP:
                        {
                            idx_t pre_idx = precomputed_feature_idx(instance_idx,f_idx,active_n,state,reward);
                            f_ret = compound_feature_values[pre_idx];
                            break;
                        }
                        case BASE_LOOK_UP:
                            f_ret = active_features[f_idx].evaluate(base_feature_values[instance_idx][state_reward_idx]);
                            break;
                        default:
                            DEBUG_DEAD_LINE;
                        }
                        /* active_state_reward_evaluations[f_idx][state_reward_idx] = f_ret; */
                        active_sumFN += lambda[f_idx]*f_ret;
                    }
                    vector<double> candidate_sumFN(candidate_n,active_sumFN); // individual terms for candidate features
                    for(uint f_idx=0; f_idx<candidate_n; ++f_idx) { // sum over features
                        // compute and store
                        f_ret_t f_ret;
                        switch(precomputation_type) {
                        case NONE:
                        case COMPOUND_LOOK_UP:
                            f_ret = candidate_features[f_idx].evaluate(insIt-1,action,state,reward);
                            break;
                        case BASE_LOOK_UP:
                            f_ret = candidate_features[f_idx].evaluate(base_feature_values[instance_idx][state_reward_idx]);
                            break;
                        default:
                            DEBUG_DEAD_LINE;
                        }
                        /* candidate_state_reward_evaluations[f_idx][state_reward_idx] = f_ret; */
                        candidate_sumFN[f_idx] += x[f_idx]*f_ret;                            // increment sumF(x(n),y')
                        candidate_sumExpN[f_idx] += exp( candidate_sumFN[f_idx] );           // increment sumExp(x(n))
                        candidate_sumFExpNF[f_idx] += f_ret * exp( candidate_sumFN[f_idx] ); // increment sumFExp(x(n),F)
                    }

                    // increment state-reward index
                    ++state_reward_idx;
                }
            }

            //----------------------------------//
            // increment objective and gradient //
            //----------------------------------//

            // objective is sum over candidate features
            for(uint f_idx=0; f_idx<candidate_n; ++f_idx) { // sum over features
                // log-probability for this datum
                double log_prob = candidate_sumFNN[f_idx] - log( candidate_sumExpN[f_idx] );

                // increment fx (objective)
                fx += log_prob;

                // increment score
                candidate_feature_scores[f_idx] += log_prob;

                // increment gradient
                g[f_idx] -= candidate_sumFExpNF[f_idx]/candidate_sumExpN[f_idx];

                // in case of parameter binding additionally sum over all features belonging to this parameter
                g[f_idx] += candidate_instance_evaluations[f_idx];
            }
        }
    }

    // terminate progress bar
    if(DEBUG_LEVEL>0) {
        ProgressBar::terminate();
    }

    // use NEGATIVE log likelihood (blfgs minimizes the objective)
    // use mean value per data point
    fx *= -1./number_of_data_points;
    for(int i=0; i<n; i++) {
        g[i] *= -1./number_of_data_points;
    }

    return fx;
}

int KMarkovCRF::static_progress_candidates(
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
    return ((KMarkovCRF*)instance)->progress_candidates(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

int KMarkovCRF::progress_candidates(
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
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", p = " << exp(-fx) << "):");
    for(uint f_idx=0; f_idx<candidate_features.size(); ++f_idx) {
        DEBUG_OUT(1, "    " <<
                candidate_features[f_idx].identifier() <<
                " --> t[" <<
                f_idx << "] = " <<
                x[f_idx]);
    }
    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", p = " << exp(-fx) << "):");
    DEBUG_OUT(1,"    Ignored " << ignored_data << " out of " << number_of_data_points << " data points");
    DEBUG_OUT(1,"");

    return 0;
}

int KMarkovCRF::optimize_candidates(lbfgsfloatval_t l1,
                                    unsigned int max_iter,
                                    lbfgsfloatval_t * mean_likelihood) {

    // Check size of parameter vector
    lbfgs_free(lambda_candidates);
    lambda_candidates = lbfgs_malloc(candidate_features.size());

    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.orthantwise_c = l1;
    param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
    if(max_iter>0) {
        param.max_iterations = max_iter;
    }

    // todo what values
    param.delta = 1e-5;   // change of objective (f-f')/f (default 0)
    param.epsilon = 1e-5; // change of parameters ||g||/max(1,||x||) (default 1e-5)

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(candidate_features.size(), lambda_candidates, &fx, static_evaluate_candidates, static_progress_candidates, this, &param);

    // Report the result.
    DEBUG_OUT(1, "L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");
    DEBUG_OUT(1,"mean likelihood = " << exp(-fx) );
    DEBUG_OUT(1,"");

    if(mean_likelihood!=nullptr) {
        *mean_likelihood = exp(-fx);
    }

    return ret;
}

void KMarkovCRF::add_action_state_reward_tripel(
    const action_t& action,
    const state_t& state,
    const reward_t& reward,
    const bool& new_episode
    ) {
    HistoryObserver::add_action_state_reward_tripel(action, state, reward, new_episode);
    feature_values_precomputed = false;
}

void KMarkovCRF::check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation) {

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

void KMarkovCRF::evaluate_features() {
    if(number_of_data_points==0) {
        DEBUG_OUT(0,"No data to evaluate features");
    } else {
        DEBUG_OUT(0,"Evaluating features:");
        for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
            DEBUG_OUT(0, "    " << active_features[f_idx].identifier() << " = " << active_features[f_idx].evaluate(instance_data.back()) );
        }
    }
}

void KMarkovCRF::construct_candidate_features(const int& n) {

    DEBUG_OUT(1, "Constructing candidate features...");

    candidate_features.clear();

    if(n<0) {
        DEBUG_OUT(0, "    Multiplicity must be non-negative");
        return;
    }
    if(n==0) {
        DEBUG_OUT(1, "    Multiplicity is zero, no features constructed");
        return;
    }

    // Temporally use list structure to make sorting and erasing more efficient
    list<AndFeature> candidate_feature_list;

    // Add active features
    for(uint f_idx = 0; f_idx < active_features.size(); ++f_idx) {
        DEBUG_OUT(2,"Including " << active_features[f_idx].identifier() << " in base set");
        candidate_feature_list.push_back(AndFeature(active_features[f_idx]));
    }

    // Add NullFeature if active features were empty
    if(candidate_feature_list.size()==0) {
        candidate_feature_list.push_back(AndFeature());
        DEBUG_OUT(2,"Used " <<  candidate_feature_list.front().identifier() << " as base");
    }

    // Replace current features by those that can
    // be reached by combining one of the current
    // features with n basis features.
    for(int order=1; order<=n; ++order) {
        int counter = candidate_feature_list.size();
        list<AndFeature>::iterator cf_it = candidate_feature_list.begin();
        while( counter>0 ) {
            for(uint bf_idx = 0; bf_idx < basis_features.size(); ++bf_idx) {
                DEBUG_OUT(2,"Candidate: " << cf_it->identifier() << ", Basis(" << bf_idx << "): " << basis_features[bf_idx]->identifier() )
                AndFeature and_feature(*basis_features[bf_idx],*cf_it);
                DEBUG_OUT(2,"    --> " << and_feature.identifier() );
                // make sure the basis feature is not already
                // part of the candidate feature (duplicates
                // are removed below)
                if(and_feature!=*cf_it) {
                    candidate_feature_list.push_back(and_feature);
                    DEBUG_OUT(2,"    accepted");
                } else {
                    DEBUG_OUT(2,"    rejected");
                }
            }
            ++cf_it;
            --counter;
            candidate_feature_list.pop_front();
        }
    }

    candidate_feature_list.sort();
    list<AndFeature>::iterator cf_it_1 = candidate_feature_list.begin();
    list<AndFeature>::iterator cf_it_2 = candidate_feature_list.begin();
    ++cf_it_2;
    while( cf_it_1!=candidate_feature_list.end() ) {
        if(cf_it_2!=candidate_feature_list.end() && *cf_it_2==*cf_it_1) {
            DEBUG_OUT(2, "    Remove " << cf_it_2->identifier() );
            cf_it_2 = candidate_feature_list.erase(cf_it_2);
        } else {
            DEBUG_OUT(2, "    Keep   " << cf_it_1->identifier() );
            candidate_features.push_back(*cf_it_1);
            ++cf_it_1;
            ++cf_it_2;
        }
    }

    DEBUG_OUT(1, "    Constructed " << candidate_features.size() << " features");

    erase_const_zero_candidate_features();

    candidate_feature_scores.assign(candidate_features.size(),0.0);
}

void KMarkovCRF::score_candidates_by_gradient() {

    DEBUG_OUT(1, "Scoring features by gradient...");

    //------------------//
    //  Check for Data  //
    //------------------//

    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to score features (" << number_of_data_points << ").");
        return;
    }

    //---------------------------------------------//
    // Compute Gradient for all Candidate Features //
    //---------------------------------------------//

    int cf_size = candidate_features.size();

    // to make the code more readable and comparable to evaluate_model() function
    vector<double> &g = candidate_feature_scores;

    // Print parameter vector
    if(DEBUG_LEVEL>=2) {
        DEBUG_OUT(2, "Parameter vector:");
        for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
            DEBUG_OUT(2, "    t[" << f_idx << "] = " << lambda[f_idx] );
        }
    }

    double sumFN; // sumF(x(n),y') is independent of candidate features since they have zero coefficient (no parameter binding!)
    double sumExpN; // normalization Z(x) is independent of candidate features since sumF(x(n),y') is independent
    vector<double> sumFExpNF(cf_size,0.0); // sumFExp(x(n),F) for all candidate features F
    idx_t instance_idx = 0;
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Scoring: ");
    }
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt, ++instance_idx) {

            if(DEBUG_LEVEL>0) {
                ProgressBar::print(instance_idx, number_of_data_points);
            }

            action_t action = insIt->action;

            sumExpN = 0.0;
            sumFExpNF.assign(cf_size,0.0);

            // calculate sumExp(x(n))
            for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
                for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID; ++reward) {

                    // calculate sumF(x(n),y')
                    sumFN = 0.0;
                    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
                        sumFN += lambda[f_idx]*active_features[f_idx].evaluate(insIt-1,action,state,reward);
                        // candidate features have zero coefficient (no parameter binding possible)!
                    }

                    // increment sumExp(x(n))
                    sumExpN += exp( sumFN );

                    // increment sumFExp(x(n),F)
                    for(int lambda_cf_idx=0; lambda_cf_idx<cf_size; ++lambda_cf_idx) { // for all parameters/gradient components (i.e. for all candidate features)
                        // in case of parameter binding additionally sum over all features belonging to this parameter (not allowed!)
                        sumFExpNF[lambda_cf_idx] += candidate_features[lambda_cf_idx].evaluate(insIt-1,action,state,reward) * exp( sumFN );
                    }
                }
            }

            // increment gradient
            for(int lambda_cf_idx=0; lambda_cf_idx<cf_size; ++lambda_cf_idx) { // for all parameters/gradient components
                g[lambda_cf_idx] -= sumFExpNF[lambda_cf_idx]/sumExpN;

                // in case of parameter binding additionally sum over all features belonging to this parameter (not allowed!)
                g[lambda_cf_idx] += candidate_features[lambda_cf_idx].evaluate(insIt);
            }
        }
    }
    if(DEBUG_LEVEL>0) {
        ProgressBar::terminate();
    }

    // use absolute mean value per data point
    for(int i=0; i<cf_size; i++) {
        g[i] = fabs(g[i])/number_of_data_points;
    }

    sort_scored_features(false);

    DEBUG_OUT(1, "DONE");
}

void KMarkovCRF::score_candidates_by_1D_optimization() {
    DEBUG_OUT(1, "Scoring features by 1D optimization...");

    //------------------//
    //  Check for Data  //
    //------------------//

    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to score features (" << number_of_data_points << ").");
        return;
    }

    //------------------------------------//
    // 1D optimization for all candidates //
    //------------------------------------//
    optimize_candidates(0, 0,nullptr);

    //-----------------//
    // sort candidates //
    //-----------------//
    for(double& score : candidate_feature_scores) { // convert scores from log-likelihood to likelihood
        score = exp(score);
    }
    sort_scored_features(false);
}

void KMarkovCRF::add_candidate_features_to_active(const int& n) {

    if(n==0) {
        DEBUG_OUT(1, "Adding all non-zero scored candidate features to active...");
    } else {
        DEBUG_OUT(1, "Adding " << n << " highest scored candidate features to active...");
    }

    // choose n highest scored candidates and erase from candidate list
    int counter = 0;
    for(int cf_idx=(int)candidate_features.size()-1; cf_idx>=0; --cf_idx) {
        if(candidate_feature_scores[cf_idx]>0) {
            active_features.push_back(candidate_features[cf_idx]);
            DEBUG_OUT(1, "added   (idx = " << cf_idx << ", score = " << candidate_feature_scores[cf_idx] << "): " << candidate_features[cf_idx].identifier());
            candidate_features.erase(candidate_features.begin() + cf_idx);
            if(++counter>=n && n>0) break;
        } else {
            DEBUG_OUT(1, "ignored (idx = " << cf_idx << ", score = " << candidate_feature_scores[cf_idx] << "): " << candidate_features[cf_idx].identifier());
        }
    }

    // need to precompute feature values for COMPOUND_LOOK_UP
    if(precomputation_type==COMPOUND_LOOK_UP) {
        feature_values_precomputed = false;
    }

    DEBUG_OUT(1, "DONE");
}

void KMarkovCRF::erase_zero_features() {

    DEBUG_OUT(1, "Erasing zero weighted features from active...");

    // Count features to erase //
    uint new_size = active_features.size();
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        if(lambda[f_idx]==0) {
            --new_size;
        }
    }

    if( new_size == active_features.size() ) {
        DEBUG_OUT(1, "DONE");
        return;
    }

    // Create new active_features, parameter_indices, and parameters (lambda)
    lbfgsfloatval_t * new_lambda = lbfgs_malloc(new_size);
    vector<AndFeature>  new_active_features(new_size,AndFeature());
    int new_f_idx = 0;
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        if(lambda[f_idx]!=0) {
            new_lambda[new_f_idx]            = lambda[f_idx];
            new_active_features[new_f_idx]   = active_features[f_idx];
            DEBUG_OUT(1,"Added " << new_active_features[new_f_idx].identifier() << " (new_idx = " << new_f_idx << ", old_idx = " << f_idx << ") to new active features");
            ++new_f_idx;
        }
    }

    // Swap new and old data
    lbfgs_free(lambda);
    lambda = new_lambda;
    active_features.swap(new_active_features);
    old_active_features_size = active_features.size();

    // need to precompute feature values for COMPOUND_LOOK_UP
    if(precomputation_type==COMPOUND_LOOK_UP) {
        feature_values_precomputed = false;
    }

    DEBUG_OUT(1, "DONE");
}

void KMarkovCRF::erase_all_features() {
    DEBUG_OUT(1, "Erasing all features from active...");

    lbfgs_free(lambda);
    lambda = nullptr;
    active_features.clear();
    old_active_features_size = 0;

    // need to precompute feature values for COMPOUND_LOOK_UP
    if(precomputation_type==COMPOUND_LOOK_UP) {
        feature_values_precomputed = false;
    }

    DEBUG_OUT(1, "DONE");
}

unsigned long KMarkovCRF::get_number_of_features() {
    return active_features.size();
}

KMarkovCRF::probability_t KMarkovCRF::get_prediction(
        const instance_t * instance,
        const action_t& action,
        const state_t& state_to,
        const reward_t& reward) const {

    // calculate sumF(x,y)
    probability_t sumFXY = 0;
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
        sumFXY += lambda[f_idx]*active_features[f_idx].evaluate(instance,action,state_to,reward);
    }

    // calculate sumExp(x)
    probability_t sumExpX = 0;
    for(stateIt_t sum_state=stateIt_t::first(); sum_state!=INVALID; ++sum_state) {
        for(rewardIt_t sum_reward=rewardIt_t::first(); sum_reward!=INVALID; ++sum_reward) {

            // calculate sumF(x,y')
            probability_t sumFXYs = 0;
            for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
                sumFXYs += lambda[f_idx]*active_features[f_idx].evaluate(instance,action,sum_state,sum_reward);
            }

            // increment sumExp(x(n))
            sumExpX += exp( sumFXYs );
        }
    }

    return exp( sumFXY )/sumExpX;
}

KMarkovCRF::probability_t KMarkovCRF::get_kmdp_prediction(
        const instance_t * instance,
        const action_t& action,
        const state_t& state_to,
        const reward_t& reward)
const {

    // check if data for input exist
    input_tuple_t input_tuple = std::make_tuple(instance, action);
    auto input_ret = input_set.find(input_tuple);

    if(input_ret==input_set.end()) { // no data for this input

        if(reward==reward_t::min_reward) {
            // uniform probability for zero reward
            return 1./(state_t::max_state-state_t::min_state+1);
        } else {
            // zero probability else
            return 0;
        }

    } else {

        // check for counts in prediction map
        prediction_tuple_t prediction_tuple = std::make_tuple(instance, action, state_to, reward);
        auto it = prediction_map.find(prediction_tuple);
        if(it==prediction_map.end()) { // no counts --> zero probability
            return 0;
        } else {
            return it->second; // probability calculated by relative counts
        }

    }
}

unsigned long int KMarkovCRF::get_training_data_length() {
    return number_of_data_points;
}

void KMarkovCRF::set_exclude_data(const double& p1, const double& p2) {
    exclude_data_1 = p1;
    exclude_data_2 = p2;
}

KMarkovCRF::probability_t KMarkovCRF::evaluate_on_excluded_data() {
    idx_t instance_idx = 0;
    probability_t log_prob = 0;
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt, ++instance_idx) {

            // if(insIt-1==INVALID) {
            //     continue;
            // }

            // exclude data
            double data_percent = double(instance_idx+1)/number_of_data_points;
            if(data_percent>exclude_data_1 && data_percent<exclude_data_2) {
                // prob *= get_prediction(insIt-1, insIt->action, insIt->state, insIt->reward);
                log_prob += log( get_prediction(insIt) );
            } else {
                continue;
            }

        }
    }

    return exp( log_prob );
}

void KMarkovCRF::update_prediction_map() {

    DEBUG_OUT(1,"Updating prediction map...");

    // clear old value
    prediction_map.clear();
    input_set.clear();

    // store count for different inputs for later normalization
    std::map<input_tuple_t, size_t > counts;

    // go through instance and count frequencies for transitions
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt) {

            // get data
            action_t action = insIt->action;
            state_t state = insIt->state;
            reward_t reward = insIt->reward;

            // increment frequency
            prediction_tuple_t predict_tuple = std::make_tuple(insIt, action, state, reward);
            auto ret_predict = prediction_map.insert(std::make_pair(predict_tuple,1)); // initialize with one count
            if(!ret_predict.second) { // if element already existed increment instead
                ret_predict.first->second += 1;
            }

            // increment counter for input
            auto input_tuple = std::make_tuple(insIt, action);
            auto ret_input = counts.insert(std::make_pair(input_tuple,1)); // initialize with one count
            if(!ret_input.second) { // if element already existed increment instead
                ret_input.first->second += 1;
            }

            // update input set
            input_set.insert(input_tuple);
        }
    }

    // normalize to get a probability distribution
    for(auto it = prediction_map.begin(); it!=prediction_map.end(); ++it) {

        const instance_t * instance = std::get<0>(it->first);
        action_t action = std::get<1>(it->first);
        auto input_tuple = std::make_tuple(instance, action);
        auto ret_input = counts.find(input_tuple);
        if(ret_input==counts.end()) {
            DEBUG_OUT(0,"Error: Item from prediction map not found within count-map");
        } else {
            it->second /= ret_input->second;
        }
    }

    DEBUG_OUT(1,"    DONE");
}

void KMarkovCRF::test() {
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt) {

            DEBUG_OUT(0,"   Prob = " << get_prediction(insIt) );
        }
    }
}

void KMarkovCRF::find_unique_feature_values() {

    // return if not enough data available
    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to evaluate features (" << number_of_data_points << ").");
        return;
    }

    // initialize progress bar
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Testing: ");
    }

    // set of unique feature values
    set<vector<vector<f_ret_t> > > feature_value_set;

    // iterate through data
    size_t instance_idx = 0;
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt, ++instance_idx) {

            // update progress bar
            if(DEBUG_LEVEL>0) {
                ProgressBar::print(instance_idx, number_of_data_points);
            }

            // current feature values
            vector<vector<f_ret_t> > feature_value_element;

            // remember action for being able to only change state and reward below
            action_t action = insIt->action;

            // iterate through all possible states and rewards (keeping action the same)
            for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
                for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID; ++reward) {

                    // add new element for this state-reward combination
                    feature_value_element.push_back(vector<f_ret_t>());

                    // iterate through all features
                    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
                        // add new return value for current feature
                        feature_value_element.back().push_back(active_features[f_idx].evaluate(insIt-1,action,state,reward));
                    }

                }
            }

            // for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
            //     active_features[f_idx].evaluate(insIt);
            // }

            // add feature value to set of unique values
            feature_value_set.insert(feature_value_element);

        }
    }

    // terminate progress bar
    if(DEBUG_LEVEL>0) {
        ProgressBar::terminate();
    }

    DEBUG_OUT(0,"Number of data points:           " << number_of_data_points );
    DEBUG_OUT(0,"Number of unique feature values: " << feature_value_set.size() );
}

void KMarkovCRF::check_lambda_size(lbfgsfloatval_t* & parameters, vector<AndFeature> & feature_vector, int old_feature_vector_size) {

    DEBUG_OUT(1, "Checking size of parameter vector...");

    if((int)feature_vector.size()!=old_feature_vector_size) {

        int new_feature_vector_size = feature_vector.size();
        DEBUG_OUT(2, "    old size = " << old_feature_vector_size);
        DEBUG_OUT(2, "    new size = " << new_feature_vector_size);

        lbfgsfloatval_t * old_parameters = parameters;

        int max_size = new_feature_vector_size>=old_feature_vector_size ? new_feature_vector_size : old_feature_vector_size;
        parameters = lbfgs_malloc(new_feature_vector_size);
        for(int f_idx=0; f_idx<max_size; ++f_idx) {
            if(f_idx<old_feature_vector_size) {
                parameters[f_idx] = old_parameters[f_idx];
            } else {
                parameters[f_idx] = 0;
            }
        }

        lbfgs_free(old_parameters);
        old_feature_vector_size = new_feature_vector_size;
        DEBUG_OUT(1, "    RESIZED");
    } else {
        DEBUG_OUT(1, "    OK");
    }
}

void KMarkovCRF::sort_scored_features(bool divide_by_complexity) {

    if(divide_by_complexity) {
        DEBUG_OUT(1, "Sorting scored features (considering complexity)...");
    } else {
        DEBUG_OUT(1, "Sorting scored features (NOT considering complexity)...");
    }

    // number of candidate features;
    int n = candidate_features.size();

    // sort indices by score
    list<pair<double,int> > scored_indices;
    for(int cf_idx=0; cf_idx<n; ++cf_idx) {
        if(divide_by_complexity) {
            scored_indices.push_back(make_pair(candidate_feature_scores[cf_idx]/candidate_features[cf_idx].get_complexity(),cf_idx));
        } else {
            scored_indices.push_back(make_pair(candidate_feature_scores[cf_idx],cf_idx));
        }
    }
    scored_indices.sort();

    // construct new feature and score lists
    vector<AndFeature> new_candidate_features(n);
    vector<double> new_candidate_feature_scores(n);
    int new_idx = 0;
    DEBUG_OUT(1, "Feature Scores:")
    for(list<pair<double,int> >::iterator it = scored_indices.begin(); it!=scored_indices.end(); ++it) {
        int old_idx = it->second;
        new_candidate_features[new_idx]       = candidate_features[old_idx];
        new_candidate_feature_scores[new_idx] = candidate_feature_scores[old_idx];
        if(divide_by_complexity) {
            DEBUG_OUT(1, "    " << QString("%1 (%2) <-- ").arg(candidate_feature_scores[old_idx],7,'f',5).arg(candidate_features[old_idx].get_complexity(),2).toStdString() << candidate_features[old_idx].identifier() );
        } else {
            DEBUG_OUT(1, "    " << QString("%1 <-- ").arg(candidate_feature_scores[old_idx],7,'f',5).toStdString() << candidate_features[old_idx].identifier() );
        }
        ++new_idx;
    }

    // swap lists
    candidate_features.swap(new_candidate_features);
    candidate_feature_scores.swap(new_candidate_feature_scores);

    DEBUG_OUT(1, "DONE");
}

KMarkovCRF::idx_t KMarkovCRF::precomputed_feature_idx(
    const idx_t& instance_idx,
    const idx_t& feature_idx,
    const idx_t& feature_n,
    const state_t& state,
    const reward_t& reward,
    const bool& use_state_and_reward
    ) {
    // block sizes
    idx_t entry_n_for_all_rewards = reward_t::reward_n;
    idx_t entry_n_for_all_states  = state_t::state_n * entry_n_for_all_rewards + 1; // plus one for entry where state and reward are ignored
    idx_t entry_n_for_all_fetures = feature_n        * entry_n_for_all_states;

    // compute index
    idx_t idx = instance_idx * entry_n_for_all_fetures;
    idx += feature_idx       * entry_n_for_all_states;

    if(!use_state_and_reward) {
        return idx;
    } else {
        idx += 1;
        idx += state.index() * entry_n_for_all_rewards;
        idx += reward.index();
        return idx;
    }
}

KMarkovCRF::idx_t KMarkovCRF::precomputed_feature_idx(
    const idx_t& instance_idx,
    const idx_t& feature_idx,
    const idx_t& feature_n
    ) {
    return precomputed_feature_idx(instance_idx,feature_idx,feature_n,state_t(),reward_t(),false);
}

KMarkovCRF::idx_t KMarkovCRF::precomputed_feature_idx(
    const idx_t& instance_idx,
    const idx_t& feature_idx,
    const idx_t& feature_n,
    const state_t& state,
    const reward_t& reward
    ) {
    return precomputed_feature_idx(instance_idx,feature_idx,feature_n,state,reward,true);
}

void KMarkovCRF::precompute_compound_feature_values() {

    DEBUG_OUT(1,"Precomputing feature values...");

    // get number of instances and features
    idx_t instance_n = number_of_data_points;
    idx_t feature_n = active_features.size();

    // resize vector
    idx_t value_n = instance_n;
    value_n *= feature_n;
    value_n *= state_t::state_n * reward_t::reward_n + 1;
    compound_feature_values.resize(value_n);

    // iterate through data
    idx_t instance_idx = 0;
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Precomputing: ");
    }
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt, ++instance_idx) {

            // print progress information
            if(DEBUG_LEVEL>0) {
                ProgressBar::print(instance_idx, instance_n);
            }

            DEBUG_OUT(3,"    Instance " << *insIt );

            // iterate through features
            action_t action = insIt->action;
            for(uint f_idx=0; f_idx<feature_n; ++f_idx) {

                // for instance itself without setting specific state and reward
                idx_t precomputed_index = precomputed_feature_idx(instance_idx,f_idx,feature_n);
                compound_feature_values[precomputed_index] = active_features[f_idx].evaluate(insIt-1, insIt->action, insIt->state, insIt->reward);
                DEBUG_OUT(3,"    Feature " << active_features[f_idx] <<
                          " --> " << compound_feature_values[precomputed_index] <<
                          " (idx=" << precomputed_index << ")"
                    );

                // with setting specific state and reward
                for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
                    for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID; ++reward) {
                        precomputed_index = precomputed_feature_idx(instance_idx,f_idx,feature_n,state,reward);
                        compound_feature_values[precomputed_index] = active_features[f_idx].evaluate(insIt-1,action,state,reward);
                        DEBUG_OUT(3,"    Feature " << active_features[f_idx] <<
                                  " with state=" << state << " reward=" << reward <<
                                  " --> " << compound_feature_values[precomputed_index] <<
                                  " (idx=" << precomputed_index << ")"
                            );
                    }
                }
            }
        }
    }

    // terminate progress bar
    if(DEBUG_LEVEL>0) {
        ProgressBar::terminate();
    }
}

void KMarkovCRF::precompute_base_feature_values() {

    DEBUG_OUT(1,"Precomputing feature values...");

    // get number of instances and features
    idx_t instance_n = number_of_data_points;
    idx_t basis_feature_n = basis_features.size();

    // clear values
    base_feature_values.clear();
    base_feature_indices.clear();

    // iterate through data
    idx_t instance_idx = 0;
    if(DEBUG_LEVEL>0) {
        ProgressBar::init("Precomputing: ");
    }
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt, ++instance_idx) {

            // print progress information
            if(DEBUG_LEVEL>0) {
                ProgressBar::print(instance_idx, instance_n);
            }
            DEBUG_OUT(3,"    Instance " << *insIt );

            // add entry on instance level
            base_feature_values.push_back(vector<Feature::look_up_map_t>());

            // remember action, state, and reward
            action_t current_action = insIt->action;
            state_t current_state = insIt->state;
            reward_t current_reward = insIt->reward;

            // iterate through states and rewards
            idx_t state_reward_idx=0;
            for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
                for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID; ++reward) {

                    // add entry on state-reward level
                    base_feature_values[instance_idx].push_back(Feature::look_up_map_t());

                    // remember index of actual configuration
                    if(state==current_state && reward==current_reward) {
                        base_feature_indices.push_back(state_reward_idx);
                    }

                    // iterate through features
                    for(uint f_idx=0; f_idx<basis_feature_n; ++f_idx) {
                        base_feature_values[instance_idx][state_reward_idx][basis_features[f_idx]] = basis_features[f_idx]->evaluate(insIt-1,current_action,state,reward);
                    }
                    ++state_reward_idx;
                }
            }
        }
    }

    // terminate progress bar
    if(DEBUG_LEVEL>0) {
        ProgressBar::terminate();
    }
}

void KMarkovCRF::erase_const_zero_candidate_features() {
    DEBUG_OUT(1, "Erasing const-zero features from candidates...");

    size_t candidate_features_n = candidate_features.size();
    size_t new_candidate_features_n = 0;

    // identify features to erase
    vector<bool> erase_feature(candidate_features_n,true);
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=INVALID; ++insIt) {
            for(uint f_idx=0; f_idx<candidate_features_n; ++f_idx) {
                if(erase_feature[f_idx] && candidate_features[f_idx].evaluate(insIt)!=0) {
                    erase_feature[f_idx]=false;
                    ++new_candidate_features_n;
                }
            }
        }
    }

    if( new_candidate_features_n == candidate_features_n ) {
        DEBUG_OUT(1, "    No const-zero features to erase");
        return;
    }

    // Create new candidate_features, parameter_indices, and parameters (lambda)
    vector<AndFeature>  new_candidate_features(new_candidate_features_n,AndFeature());
    int new_f_idx = 0;
    for(uint f_idx=0; f_idx<candidate_features_n; ++f_idx) {
        if(!erase_feature[f_idx]) {
            new_candidate_features[new_f_idx] = candidate_features[f_idx];
            DEBUG_OUT(2,"Added " << new_candidate_features[new_f_idx].identifier() << " (new_idx = " << new_f_idx << ", old_idx = " << f_idx << ") to new candidate features");
            ++new_f_idx;
        }
    }

    // Swap new and old data
    candidate_features.swap(new_candidate_features);

    DEBUG_OUT(1, "    Reduced from " << candidate_features_n << " to " <<
              new_candidate_features_n << " non-const-zero features"
        );
}
