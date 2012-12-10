
#include "KMarkovCRF.h"

#include "list"
#include "tuple"

#include "lbfgs_codes.h"

#define DEBUG_STRING "CRF: "
#define DEBUG_LEVEL 1
#include "debug.h"

using std::vector;
using std::list;
using std::tuple;
using std::make_tuple;
using std::get;
using std::pair;
using std::make_pair;
using std::set;

KMarkovCRF::KMarkovCRF(): k(Data::k), old_active_features_size(0), lambda(nullptr), compound_features_sorted(false) {

    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-k; --k_idx) {
        // actions
        for(action_t action=0; action<Data::action_n; ++action) {
            ActionFeature * action_feature = ActionFeature::create(action,k_idx);
            basis_features.push_back(action_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // states
        for(state_t state=0; state<Data::state_n; ++state) {
            StateFeature * state_feature = StateFeature::create(state,k_idx);
            basis_features.push_back(state_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // reward
        for(reward_t reward = Data::min_reward; reward<=Data::max_reward; reward+=Data::reward_increment) {
            RewardFeature * reward_feature = RewardFeature::create(reward,k_idx);
            basis_features.push_back(reward_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
    }
}

KMarkovCRF::~KMarkovCRF() {
    lbfgs_free(lambda);
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

    int number_of_data_points = episode_data.size()-k;
    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to evaluate model.");
        return fx;
    }

    // Print parameter vector //
    if(DEBUG_LEVEL>=2) {
        DEBUG_OUT(2, "Parameter vector:");
        for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
            DEBUG_OUT(2, "    t[" << f_idx << "] = " << x[f_idx] );
        }
    }

    double sumFNN; // sumF(x(n),y(n))
    double sumExpN; // normalization Z(x)
    vector<double> sumFExpNF(n,0.0); // sumFExp(x(n),F)
    for(const_episode_iterator_t episode_iterator=episode_data.begin()+k;
            episode_iterator!=episode_data.end();
            ++episode_iterator) {

        sumFNN = 0;
        sumExpN = 0;
        sumFExpNF.assign(n,0.0);

        // calculate sumF(x(n),y(n))
        for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
            sumFNN += x[f_idx]*active_features[f_idx].evaluate(episode_iterator);
        }

        // calculate sumExp(x(n))
        for(OutputIterator output_iterator; !output_iterator.end(); ++output_iterator) {

            // calculate sumF(x(n),y')
            double sumFN = 0;
            for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
                sumFN += x[f_idx]*active_features[f_idx].evaluate(episode_iterator,*output_iterator);
            }

            // increment sumExp(x(n))
            sumExpN += exp( sumFN );

            // increment sumFExp(x(n),F)
            for(int lambda_idx=0; lambda_idx<n; ++lambda_idx) { // for all parameters/gradient components
                // in case of parameter binding additionally sum over all features belonging to this parameter
                sumFExpNF[lambda_idx] += active_features[lambda_idx].evaluate(episode_iterator,*output_iterator) * exp( sumFN );
            }
        }

        // increment fx
        fx += sumFNN - log( sumExpN );

        // increment gradient
        for(int lambda_idx=0; lambda_idx<n; ++lambda_idx) { // for all parameters/gradient components
            g[lambda_idx] -= sumFExpNF[lambda_idx]/sumExpN;

            // in case of parameter binding additionally sum over all features belonging to this parameter
            g[lambda_idx] += active_features[lambda_idx].evaluate(episode_iterator);
        }
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
    DEBUG_OUT(0,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", p = " << exp(-fx) << "):");
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        DEBUG_OUT(1, "    " <<
                active_features[f_idx].identifier() <<
                " --> t[" <<
                f_idx << "] = " <<
                x[f_idx]);
    }
    DEBUG_OUT(0,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", p = " << exp(-fx) << "):");
    DEBUG_OUT(1,"");

    return 0;
}

void KMarkovCRF::optimize_model(lbfgsfloatval_t l1) {

    // Check size of parameter vector //
    check_lambda_size();

    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.orthantwise_c = l1;
    param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
    param.max_iterations = 20;

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(active_features.size(), lambda, &fx, static_evaluate_model, static_progress_model, this, &param);

    // Report the result.
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx) );
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        DEBUG_OUT(1, "    " <<
                active_features[f_idx].identifier() <<
                " --> t[" <<
                f_idx << "] = " <<
                lambda[f_idx]);
    }
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx) );
    DEBUG_OUT(0,"");
}

void KMarkovCRF::add_action_state_reward_tripel(
        const action_t& action,
        const state_t& state,
        const reward_t& reward
) {
    episode_data.push_back(data_point_t(action,state,reward));
    DEBUG_OUT(1, "added (action,state,reward) = (" << action << "," << state << "," << reward << ")" );
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

            // print result
            lbfgsfloatval_t ngrad = (fx_plus-fx_minus)/dx[x_idx];
            DEBUG_OUT(1,
                    "    diff[" << x_idx << "] = " << grad[x_idx]-ngrad <<
                    ", grad["   << x_idx << "] = " << grad[x_idx] <<
                    ", ngrad["  << x_idx << "] = " << ngrad <<
                    ", x["      << x_idx << "] = " << x[x_idx] <<
                    ", dx["     << x_idx << "] = " << dx[x_idx]
            );

            // check for deviations
            if(fabs(ngrad-grad[x_idx])/fabs(grad[x_idx])>relative_deviation) {
                relative_deviation=fabs(ngrad-grad[x_idx])/fabs(grad[x_idx]);
            }
        }
    }
    if(relative_deviation>max_relative_deviation) {
        DEBUG_OUT(0, "ERRORS in first derivative found: max relative deviation = " << relative_deviation << "(tolerance = " << max_relative_deviation << ")" );
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
    int number_of_data_points = episode_data.size()-k;
    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to evaluate model.");
        return;
    }

    DEBUG_OUT(0,"Evaluating features:");
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        DEBUG_OUT(0, "    " << active_features[f_idx].identifier() << " = " << active_features[f_idx].evaluate(episode_data.end()-1) );
    }
}

//void KMarkovCRF::score_features_by_mutual_information() {
//
//    DEBUG_OUT(1, "Scoring features by mutual information...");
//
//    //------------------//
//    //  Check for Data  //
//    //------------------//
//
//    int N = episode_data.size()-k;
//    if(N<=0) {
//        DEBUG_OUT(0,"Not enough data to score features.");
//        return;
//    }
//
//    //---------------------//
//    // Construct Features  //
//    //---------------------//
//
//    construct_compound_features(2);
//
//    //---------------------------------//
//    // Determine Relative Frequencies  //
//    //---------------------------------//
//
//    vector<int>          output_counts (Data::output_n                                      ,0  );
//    vector<int>          feature_counts(compound_features.size()                            ,0  );
//    vector<vector<int> > joint_counts  (Data::output_n, vector<int>(compound_features.size(),0) );
//
//    for(const_episode_iterator_t episode_iterator=episode_data.begin()+k;
//            episode_iterator!=episode_data.end();
//            ++episode_iterator) {
//        int output_idx = Data::output_idx(episode_iterator);
//        output_counts[output_idx] += 1;
//        for(uint f_idx=0; f_idx<compound_features.size(); ++f_idx) {
//            if(compound_features[f_idx].evaluate(episode_iterator)) {
//                feature_counts[f_idx]           += 1;
//                joint_counts[output_idx][f_idx] += 1;
//            }
//        }
//    }
//
//    //----------------------------------------------------------//
//    // Compute Mutual Information for all Features with Outputs //
//    //----------------------------------------------------------//
//
//    for(uint cf_idx=0; cf_idx<compound_features.size(); ++cf_idx) {
//        for(int output_idx=0; output_idx<Data::output_n; ++output_idx) {
//            double po, pof1, pf1, d1, p1, p2, q1, q2, l1, l2;
//            po = (double)output_counts[output_idx]/N;         // p(o)
//            pof1 = (double)joint_counts[output_idx][cf_idx]/N; // p(o,f=1)
//            pf1 = (double)feature_counts[cf_idx]/N;            // p(f=1)
//            d1 = p1 = p2 = po;
//            d1 -= pof1;
//            p1 *= 1 - pf1;
//            p2 *= pf1;
//            q1 = d1/p1;
//            q2 = l2 = pof1;
//            q2 /= p2;
//            l1 = d1 * log(q1);
//            l2 *= log(q2);
//            if(d1==0) {
//                l1 = 0;
//            }
//            if(pof1==0) {
//                l2 = 0;
//            }
//            compound_feature_scores[cf_idx] += l1 + l2;
//        }
//    }
//
//    compound_features_sorted = false;
//
//    //----------------------//
//    // Sort Feature Scores  //
//    //----------------------//
//
//    //    typedef list<tuple<double,unsigned int,AndFeature> > score_list;
//    //    score_list sorted_feature_scores(feature_scores.size());
//    //    int f_idx = 0;
//    //    for(score_list::iterator it = sorted_feature_scores.begin();
//    //            it!=sorted_feature_scores.end();
//    //            ++it) {
//    //        unsigned int complexity = compound_features[f_idx].get_complexity();
//    //        (*it) = make_tuple(feature_scores[f_idx]/complexity,complexity,compound_features[f_idx]); // mutual information divided by feature complexity
//    //        ++f_idx;
//    //    }
//    //    sorted_feature_scores.sort();
//    //
//    //    //----------------------//
//    //    // Print Feature Scores //
//    //    //----------------------//
//    //
//    //    DEBUG_OUT(0,"Feature Scores:");
//    //    for(score_list::const_iterator it = sorted_feature_scores.begin();
//    //            it!=sorted_feature_scores.end();
//    //            ++it) {
//    //        DEBUG_OUT(0,"    " << QString("%1 (%2) <-- ").arg(get<0>(*it),7,'f',5).arg(get<1>(*it),2).toStdString() << get<2>(*it).identifier() );
//    //    }
//    //
//    //    //------------------------//
//    //    // Add to Active Features //
//    //    //------------------------//
//    //    active_features.clear();
//    //    lbfgs_free(lambda);
//    //    lambda = nullptr;
//    //    old_active_features_size = 0;
//    //
//    //    for(uint f_idx=0; f_idx<compound_features.size(); ++f_idx) {
//    //        if(feature_scores[f_idx]>0) {
//    //            active_features.push_back(compound_features[f_idx]);
//    //            DEBUG_OUT(0, "added   (idx = " << f_idx << ", score = " << feature_scores[f_idx] << "): " << compound_features[f_idx].identifier());
//    //        } else {
//    //            DEBUG_OUT(0, "ignored (idx = " << f_idx << ", score = " << feature_scores[f_idx] << "): " << compound_features[f_idx].identifier());
//    //        }
//    //    }
//
//    DEBUG_OUT(1, "DONE");
//
//}

void KMarkovCRF::score_features_by_gradient(const int& n) {

    DEBUG_OUT(1, "Scoring features by gradient...");

    //------------------//
    //  Check for Data  //
    //------------------//

    int N = episode_data.size()-k;
    if(N<=0) {
        DEBUG_OUT(0,"Not enough data to score features.");
        return;
    }

    //---------------------//
    // Construct Features  //
    //---------------------//

    construct_compound_features(n);

    //--------------------------------------------//
    // Compute Gradient for all Compound Features //
    //--------------------------------------------//

    int cf_size = compound_features.size();

    // to make the code more readable and comparable to evaluate_model() function
    vector<double> &g = compound_feature_scores;

    int number_of_data_points = episode_data.size()-k;
    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to evaluate features.");
        return;
    }

    // Print parameter vector //
    if(DEBUG_LEVEL>=2) {
        DEBUG_OUT(2, "Parameter vector:");
        for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
            DEBUG_OUT(2, "    t[" << f_idx << "] = " << lambda[f_idx] );
        }
    }


    double sumFN; // sumF(x(n),y') is independent of compound features since they have zero coefficient (no parameter binding!)
    double sumExpN; // normalization Z(x) is independent of compound features since sumF(x(n),y') is independent
    vector<double> sumFExpNF(cf_size,0.0); // sumFExp(x(n),F) for all compound features F
    for(const_episode_iterator_t episode_iterator=episode_data.begin()+k;
            episode_iterator!=episode_data.end();
            ++episode_iterator) {

        sumExpN = 0.0;
        sumFExpNF.assign(cf_size,0.0);

        // calculate sumExp(x(n))
        for(OutputIterator output_iterator; !output_iterator.end(); ++output_iterator) {

            // calculate sumF(x(n),y')
            sumFN = 0.0;
            for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
                sumFN += lambda[f_idx]*active_features[f_idx].evaluate(episode_iterator,*output_iterator);
                // compound features have zero coefficient (no parameter binding possible)!
            }

            // increment sumExp(x(n))
            sumExpN += exp( sumFN );

            // increment sumFExp(x(n),F)
            for(int lambda_cf_idx=0; lambda_cf_idx<cf_size; ++lambda_cf_idx) { // for all parameters/gradient components (i.e. for all compound features)
                // in case of parameter binding additionally sum over all features belonging to this parameter (not allowed!)
                sumFExpNF[lambda_cf_idx] += compound_features[lambda_cf_idx].evaluate(episode_iterator,*output_iterator) * exp( sumFN );
            }
        }

        // increment gradient
        for(int lambda_cf_idx=0; lambda_cf_idx<cf_size; ++lambda_cf_idx) { // for all parameters/gradient components
            g[lambda_cf_idx] -= sumFExpNF[lambda_cf_idx]/sumExpN;

            // in case of parameter binding additionally sum over all features belonging to this parameter (not allowed!)
            g[lambda_cf_idx] += compound_features[lambda_cf_idx].evaluate(episode_iterator);
        }
    }

    // use absolute mean value per data point
    for(int i=0; i<cf_size; i++) {
        g[i] = fabs(g[i])/number_of_data_points;
    }

    compound_features_sorted = false;

    DEBUG_OUT(1, "DONE");
}

void KMarkovCRF::sort_scored_features(bool divide_by_complexity) {

    if(divide_by_complexity) {
        DEBUG_OUT(1, "Sorting scored features (considering complexity)...");
    } else {
        DEBUG_OUT(1, "Sorting scored features (NOT considering complexity)...");
    }

    // number of compound features;
    int n = compound_features.size();

    // sort indices by score
    list<pair<double,int> > scored_indices;
    for(int cf_idx=0; cf_idx<n; ++cf_idx) {
        if(divide_by_complexity) {
            scored_indices.push_back(make_pair(compound_feature_scores[cf_idx]/compound_features[cf_idx].get_complexity(),cf_idx));
        } else {
            scored_indices.push_back(make_pair(compound_feature_scores[cf_idx],cf_idx));
        }
    }
    scored_indices.sort();

    // construct new feature and score lists
    vector<AndFeature> new_compound_features(n);
    vector<double> new_compound_feature_scores(n);
    int new_idx = 0;
    DEBUG_OUT(1, "Feature Scores:")
    for(list<pair<double,int> >::iterator it = scored_indices.begin(); it!=scored_indices.end(); ++it) {
        int old_idx = it->second;
        new_compound_features[new_idx]       = compound_features[old_idx];
        new_compound_feature_scores[new_idx] = compound_feature_scores[old_idx];
        if(divide_by_complexity) {
            DEBUG_OUT(1, "    " << QString("%1 (%2) <-- ").arg(compound_feature_scores[old_idx],7,'f',5).arg(compound_features[old_idx].get_complexity(),2).toStdString() << compound_features[old_idx].identifier() );
        } else {
            DEBUG_OUT(1, "    " << QString("%1 <-- ").arg(compound_feature_scores[old_idx],7,'f',5).toStdString() << compound_features[old_idx].identifier() );
        }
        ++new_idx;
    }

    // swap lists
    compound_features.swap(new_compound_features);
    compound_feature_scores.swap(new_compound_feature_scores);

    compound_features_sorted = true;

    DEBUG_OUT(1, "DONE");
}

void KMarkovCRF::add_compound_features_to_active(const int& n) {

    if(n==0) {
        DEBUG_OUT(1, "Adding all non-zero scored compound features to active...");
    } else {
        DEBUG_OUT(1, "Adding " << n << " highest scored compound features to active...");
    }

    if(!compound_features_sorted) sort_scored_features();

    int counter = 0;
    for(int cf_idx=(int)compound_features.size()-1; cf_idx>=0; --cf_idx) {
        if(compound_feature_scores[cf_idx]>0) {
            active_features.push_back(compound_features[cf_idx]);
            DEBUG_OUT(1, "added   (idx = " << cf_idx << ", score = " << compound_feature_scores[cf_idx] << "): " << compound_features[cf_idx].identifier());
            if(++counter>=n && n>0) break;
        } else {
            DEBUG_OUT(1, "ignored (idx = " << cf_idx << ", score = " << compound_feature_scores[cf_idx] << "): " << compound_features[cf_idx].identifier());
        }
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

    DEBUG_OUT(1, "DONE");
}

KMarkovCRF::probability_t KMarkovCRF::prediction(
        const k_mdp_state_t& state_from,
        const action_t& action,
        const state_t& state_to,
        const reward_t& reward) {

    episode_t episode(state_from.size());
    for(unsigned int idx=0; idx<state_from.size(); ++idx) {
        episode[state_from.size()-idx-1] = state_from[idx];
    }
    episode.push_back(data_point_t(action,state_to,reward));

    input_data_t input_data = episode.end();
    --input_data;

    // calculate sumF(x,y)
    probability_t sumFXY = 0;
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
        sumFXY += lambda[f_idx]*active_features[f_idx].evaluate(input_data);
    }

    // calculate sumExp(x)
    probability_t sumExpX = 0;
    for(OutputIterator output_iterator; !output_iterator.end(); ++output_iterator) {

        // calculate sumF(x,y')
        probability_t sumFXYs = 0;
        for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
            sumFXYs += lambda[f_idx]*active_features[f_idx].evaluate(input_data,*output_iterator);
        }

        // increment sumExp(x(n))
        sumExpX += exp( sumFXYs );
    }

    return exp( sumFXY )/sumExpX;
}

void KMarkovCRF::initialize_sparse_predictions(QIteration& predictions) {

    unsigned long counter = 0;

    for(Data::k_mdp_state_idx_t state_from_idx=0;
            state_from_idx<Data::k_mdp_state_n;
            ++state_from_idx) {

        k_mdp_state_t state_from = Data::k_mdp_state_from_idx(state_from_idx);

        for(Data::action_t action = 0; action<Data::action_n; ++action) {

            for(OutputIterator it; !it.end(); ++it) {

                state_t state_to = (*it).state;
                reward_t reward  = (*it).reward;

                predictions.set_prediction(
                        state_from,
                        action,
                        state_to,
                        reward,
                        prediction(state_from,action,state_to,reward)
                );

                ++counter;
            }

        }
    }
    DEBUG_OUT(1,"Initialized " << counter << " predictions");
}

void KMarkovCRF::initialize_kmdp_predictions(QIteration& predictions) {

    int number_of_data_points = episode_data.size()-k;
    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to evaluate model.");
    }

    //--------------------------------//
    // determine relative frequencies //
    //--------------------------------//
    std::vector<unsigned long> counts(Data::k_mdp_state_n*Data::action_n*Data::state_n*Data::reward_n,0);
    for(const_episode_iterator_t episode_iterator=episode_data.begin()+k;
            episode_iterator!=episode_data.end();
            ++episode_iterator) {

        k_mdp_state_t k_mdp_state(Data::k);
        for(unsigned int idx=0; idx<k_mdp_state.size(); ++idx) {
            k_mdp_state[idx] = *(episode_iterator-idx-1);
        }
        action_t action = episode_iterator->action;
        state_t state = episode_iterator->state;
        reward_t reward = episode_iterator->reward;
        counts[Data::prediction_idx(k_mdp_state,action,state,reward)] += 1;
    }

    //-----------------------------//
    // assign relative frequencies //
    //-----------------------------//
    unsigned long counter = 0;
    for(Data::k_mdp_state_idx_t state_from_idx=0;
            state_from_idx<Data::k_mdp_state_n;
            ++state_from_idx) {

        k_mdp_state_t state_from = Data::k_mdp_state_from_idx(state_from_idx);

        for(Data::action_t action = 0; action<Data::action_n; ++action) {

            for(OutputIterator it; !it.end(); ++it) {

                state_t state_to = (*it).state;
                reward_t reward  = (*it).reward;

                predictions.set_prediction(
                        state_from,
                        action,
                        state_to,
                        reward,
                        (probability_t)counts[Data::prediction_idx(state_from,action,state_to,reward)]/number_of_data_points
                );

                ++counter;
            }

        }
    }
    DEBUG_OUT(1,"Initialized " << counter << " predictions");
}

void KMarkovCRF::check_lambda_size() {

    DEBUG_OUT(1, "Checking size of parameter vector...");

    if((int)active_features.size()!=old_active_features_size) {

        int new_active_features_size = active_features.size();
        DEBUG_OUT(2, "    old size = " << old_active_features_size);
        DEBUG_OUT(2, "    new size = " << new_active_features_size);

        lbfgsfloatval_t * old_lambda = lambda;

        int max_size = new_active_features_size>=old_active_features_size ? new_active_features_size : old_active_features_size;
        lambda = lbfgs_malloc(new_active_features_size);
        for(int f_idx=0; f_idx<max_size; ++f_idx) {
            if(f_idx<old_active_features_size) {
                lambda[f_idx] = old_lambda[f_idx];
            } else {
                lambda[f_idx] = 0;
            }
        }

        lbfgs_free(old_lambda);
        old_active_features_size = new_active_features_size;
        DEBUG_OUT(1, "    RESIZED");
    } else {
        DEBUG_OUT(1, "    OK");
    }
}

void KMarkovCRF::construct_compound_features(const int& n) {

    DEBUG_OUT(1, "Constructing compound features...");

    compound_features.clear();

    if(n<0) {
        DEBUG_OUT(0, "    Multiplicity must be non-negative");
        return;
    }
    if(n==0) {
        DEBUG_OUT(1, "    Multiplicity is zero, no features constructed");
        return;
    }

    // Temporally use list structure to make sorting and erasing more efficient
    list<AndFeature> compound_feature_list;

    // Add active features
    for(uint f_idx = 0; f_idx < active_features.size(); ++f_idx) {
        DEBUG_OUT(2,"Including " << active_features[f_idx].identifier() << " in base set");
        compound_feature_list.push_back(AndFeature(active_features[f_idx]));
    }

    // Add NullFeature if active features were empty
    if(compound_feature_list.size()==0) {
        compound_feature_list.push_back(AndFeature());
        DEBUG_OUT(2,"Used " <<  compound_feature_list.front().identifier() << " as base");
    }

    // Replace current features by those that can
    // be reached by combining one of the current
    // features with n basis features.
    for(int order=1; order<=n; ++order) {
        int counter = compound_feature_list.size();
        list<AndFeature>::iterator cf_it = compound_feature_list.begin();
        while( counter>0 ) {
            for(uint bf_idx = 0; bf_idx < basis_features.size(); ++bf_idx) {
                DEBUG_OUT(2,"Compound: " << cf_it->identifier() << ", Basis(" << bf_idx << "): " << basis_features[bf_idx]->identifier() )
                AndFeature and_feature(*basis_features[bf_idx],*cf_it);
                DEBUG_OUT(2,"    --> " << and_feature.identifier() );
                // make sure the basis feature is not already
                // part of the compound feature (duplicates
                // are removed below)
                if(and_feature!=*cf_it) {
                    compound_feature_list.push_back(and_feature);
                    DEBUG_OUT(2,"    accepted");
                } else {
                    DEBUG_OUT(2,"    rejected");
                }
            }
            ++cf_it;
            --counter;
            compound_feature_list.pop_front();
        }
    }

    compound_feature_list.sort();
    list<AndFeature>::iterator cf_it_1 = compound_feature_list.begin();
    list<AndFeature>::iterator cf_it_2 = compound_feature_list.begin();
    ++cf_it_2;
    while( cf_it_1!=compound_feature_list.end() ) {
        if(cf_it_2!=compound_feature_list.end() && *cf_it_2==*cf_it_1) {
            DEBUG_OUT(2, "    Remove " << cf_it_2->identifier() );
            cf_it_2 = compound_feature_list.erase(cf_it_2);
        } else {
            DEBUG_OUT(2, "    Keep   " << cf_it_1->identifier() );
            compound_features.push_back(*cf_it_1);
            ++cf_it_1;
            ++cf_it_2;
        }
    }

    DEBUG_OUT(1, "    Constructed " << compound_features.size() << " features");

    compound_feature_scores.assign(compound_features.size(),0.0);

    DEBUG_OUT(1, "DONE");
}
