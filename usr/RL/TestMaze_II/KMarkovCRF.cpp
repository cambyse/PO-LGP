
#include "KMarkovCRF.h"

#include "list"
#include "tuple"

#define DEBUG_STRING ""
#define DEBUG_LEVEL 1
#include "debug.h"

using std::vector;
using std::list;
using std::tuple;
using std::make_tuple;
using std::get;
using std::pair;
using std::set;

KMarkovCRF::KMarkovCRF( const int& k_ ): k(k_), old_active_features_size(0), lambda(nullptr) {

    //----------------------------------------//
    // Constructing basic indicator features  //
    //----------------------------------------//
    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-k; --k_idx) {
        // actions
        for(action_t action=0; action<Data::action_n; ++action) {
            ActionFeature * action_feature = new ActionFeature(action,k_idx);
            basic_features.push_back(action_feature);
        }
        // states
        for(state_t state=0; state<Data::state_n; ++state) {
            StateFeature * state_feature = new StateFeature(state,k_idx);
            basic_features.push_back(state_feature);
        }
        // reward
        for(reward_t reward = Data::min_reward; reward<=Data::max_reward; reward+=Data::reward_increment) {
            RewardFeature * reward_feature = new RewardFeature(reward,k_idx);
            basic_features.push_back(reward_feature);
        }
    }
}

KMarkovCRF::~KMarkovCRF() {
    lbfgs_free(lambda);
    for(uint f_idx=0; f_idx<basic_features.size(); ++f_idx) {
        delete basic_features[f_idx];
    }
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        delete active_features[f_idx];
    }
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

    double sumFNN;
    double sumExpN;
    vector<double> sumFExpNF(n,0.0);
    for(const_episode_iterator_t episode_iterator=episode_data.begin()+k;
            episode_iterator!=episode_data.end();
            ++episode_iterator) {

        sumFNN = 0;
        sumExpN = 0;
        sumFExpNF.assign(n,0.0);

        // calculate sumF(x(n),y(n))
        for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
            sumFNN += x[f_idx]*active_features[f_idx]->evaluate(episode_iterator);
        }

        // calculate sumExp(x(n))
        for(OutputIterator output_iterator; !output_iterator.end(); ++output_iterator) {

            // calculate sumF(x(n),y')
            double sumFN = 0;
            for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) { // sum over features
                sumFN += x[f_idx]*active_features[f_idx]->evaluate(episode_iterator,*output_iterator);
            }

            // increment sumExp(x(n))
            sumExpN += exp( sumFN );

            // increment sumFExp(x(n),F)
            for(int lambda_idx=0; lambda_idx<n; ++lambda_idx) { // for all parameters/gradient components
                // in case of parameter binding additionally sum over all features belonging to this parameter
                sumFExpNF[lambda_idx] += active_features[lambda_idx]->evaluate(episode_iterator,*output_iterator) * exp( sumFN );
            }
        }

        // increment fx
        fx += sumFNN - log( sumExpN );

        // increment gradient
        for(int lambda_idx=0; lambda_idx<n; ++lambda_idx) { // for all parameters/gradient components
            g[lambda_idx] -= sumFExpNF[lambda_idx]/sumExpN;

            // in case of parameter binding additionally sum over all features belonging to this parameter
            g[lambda_idx] += active_features[lambda_idx]->evaluate(episode_iterator);
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
                active_features[f_idx]->identifier() <<
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
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx) );
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        DEBUG_OUT(1, "    " <<
                active_features[f_idx]->identifier() <<
                " --> t[" <<
                f_idx << "] = " <<
                lambda[f_idx]);
    }
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx) );
    DEBUG_OUT(0,"");
}

void KMarkovCRF::add_action_state_reward_tripel(
        const int& action,
        const int& state,
        const double& reward
) {
    episode_data.push_back(std::make_tuple(action,state,reward));
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

//KMarkovCRF::probability_t KMarkovCRF::probability(input_data_t input_data) {
//    probability_t raw_log_probability = 0;
//
//    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
//        double f_value = active_features[f_idx]->evaluate(input_data);
//        raw_log_probability += lambda[parameter_indices[f_idx]]*f_value;
//    }
//
//    probability_t normalization = 0;
//
//    return exp(raw_log_probability)/normalization;
//}
//
//KMarkovCRF::probability_t KMarkovCRF::probability(input_data_t input_data, output_data_t output_data) {
//
//}

void KMarkovCRF::evaluate_features() {
    int number_of_data_points = episode_data.size()-k;
    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to evaluate model.");
        return;
    }

    DEBUG_OUT(0,"Evaluating features:");
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        DEBUG_OUT(0, "    " << active_features[f_idx]->identifier() << " = " << active_features[f_idx]->evaluate(episode_data.end()-1) );
    }
}

void KMarkovCRF::score_features() {

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

    vector<Feature*> compound_features;

    // from basic features //
    for(uint f1_idx=0; f1_idx<basic_features.size(); ++f1_idx) {
        AndFeature * and_feature = new AndFeature((*basic_features[f1_idx]));
        compound_features.push_back(and_feature); // include single basic features
        for(uint f2_idx=f1_idx+1; f2_idx<basic_features.size(); ++f2_idx) { // order does not matter
            AndFeature * and_feature = new AndFeature((*basic_features[f1_idx]),(*basic_features[f2_idx]));
            compound_features.push_back(and_feature);
        }
    }

    // from basic and active features //
    for(uint f1_idx=0; f1_idx<basic_features.size(); ++f1_idx) {
        for(uint f2_idx=0; f2_idx<active_features.size(); ++f2_idx) { // order DOES not matter
            AndFeature * and_feature = new AndFeature((*basic_features[f1_idx]),(*active_features[f2_idx]));
            compound_features.push_back(and_feature);
        }
    }

    // from active features //
    for(uint f1_idx=0; f1_idx<active_features.size(); ++f1_idx) {
        AndFeature * and_feature = new AndFeature((*active_features[f1_idx]));
        compound_features.push_back(and_feature); // include single active features
        for(uint f2_idx=f1_idx+1; f2_idx<active_features.size(); ++f2_idx) { // order does not matter
            AndFeature * and_feature = new AndFeature((*active_features[f1_idx]),(*active_features[f2_idx]));
            compound_features.push_back(and_feature);
        }
    }

    //---------------------------------//
    // Determine Relative Frequencies  //
    //---------------------------------//

    vector<int>          output_counts (Data::output_n                                      ,0  );
    vector<int>          feature_counts(compound_features.size()                            ,0  );
    vector<vector<int> > joint_counts  (Data::output_n, vector<int>(compound_features.size(),0) );

    for(const_episode_iterator_t episode_iterator=episode_data.begin()+k;
            episode_iterator!=episode_data.end();
            ++episode_iterator) {
        int output_idx = Data::output_idx(episode_iterator);
        output_counts[output_idx] += 1;
        for(uint f_idx=0; f_idx<compound_features.size(); ++f_idx) {
            if(compound_features[f_idx]->evaluate(episode_iterator)) {
                feature_counts[f_idx]           += 1;
                joint_counts[output_idx][f_idx] += 1;
            }
        }
    }

    //---------------------------//
    // Determine Feature Scores  //
    //---------------------------//

    vector<double> feature_scores(compound_features.size(),0);

    for(uint f_idx=0; f_idx<compound_features.size(); ++f_idx) {
        for(int output_idx=0; output_idx<Data::output_n; ++output_idx) {
            double po, pof1, pf1, d1, p1, p2, q1, q2, l1, l2;
            po = (double)output_counts[output_idx]/N;         // p(o)
            pof1 = (double)joint_counts[output_idx][f_idx]/N; // p(o,f=1)
            pf1 = (double)feature_counts[f_idx]/N;            // p(f=1)
            d1 = p1 = p2 = po;
            d1 -= pof1;
            p1 *= 1 - pf1;
            p2 *= pf1;
            q1 = d1/p1;
            q2 = l2 = pof1;
            q2 /= p2;
            l1 = d1 * log(q1);
            l2 *= log(q2);
            if(d1==0) {
                l1 = 0;
            }
            if(pof1==0) {
                l2 = 0;
            }
            feature_scores[f_idx] += l1 + l2;
        }
    }

    //----------------------//
    // Sort Feature Scores  //
    //----------------------//

    typedef list<tuple<double,unsigned int,Feature*> > score_list;
    score_list sorted_feature_scores(feature_scores.size());
    int f_idx = 0;
    for(score_list::iterator it = sorted_feature_scores.begin();
            it!=sorted_feature_scores.end();
            ++it) {
        unsigned int complexity = compound_features[f_idx]->get_complexity();
        (*it) = make_tuple(feature_scores[f_idx]/complexity,complexity,compound_features[f_idx]); // mutual information divided by feature complexity
        ++f_idx;
    }
    sorted_feature_scores.sort();

    //----------------------//
    // Print Feature Scores //
    //----------------------//

    DEBUG_OUT(0,"Feature Scores:")
    for(score_list::const_iterator it = sorted_feature_scores.begin();
            it!=sorted_feature_scores.end();
            ++it) {
        DEBUG_OUT(0,"    " << QString("%1 (%2) <-- ").arg(get<0>(*it),7,'f',5).arg(get<1>(*it),2).toStdString() << get<2>(*it)->identifier() );
    }

    //------------------------//
    // Add to Active Features //
    //------------------------//
    // todo what happens to the unmanaged features, they need to be deleted!
    active_features.clear();
    lbfgs_free(lambda);
    lambda = nullptr;
    old_active_features_size = 0;

    for(uint f_idx=0; f_idx<compound_features.size(); ++f_idx) {
        if(feature_scores[f_idx]>0) {
            active_features.push_back(compound_features[f_idx]);
            DEBUG_OUT(0, "added   (idx = " << f_idx << ", score = " << feature_scores[f_idx] << "): " << compound_features[f_idx]->identifier());
            compound_features[f_idx] = nullptr;
        } else {
            DEBUG_OUT(0, "ignored (idx = " << f_idx << ", score = " << feature_scores[f_idx] << "): " << compound_features[f_idx]->identifier());
            delete compound_features[f_idx];
            compound_features[f_idx] = nullptr;
        }
    }

    if(DEBUG_LEVEL>=1) {
        for(uint f_idx=0; f_idx<compound_features.size(); ++f_idx) {
            if(compound_features[f_idx]!=nullptr) {
                DEBUG_OUT(0, "Found undeleted compound feature (idx = " << f_idx << ")! --> deleting");
                delete compound_features[f_idx];
            }
        }
    }
}

void KMarkovCRF::erase_zero_features() {

    // Count features to erase //
    int new_size = active_features.size();
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        if(lambda[f_idx]==0) {
            --new_size;
        }
    }

    // Create new active_features, parameter_indices, and parameters (lambda)
    lbfgsfloatval_t * new_lambda = lbfgs_malloc(new_size);
    vector<Feature*>  new_active_features(new_size,nullptr);
    int new_f_idx = 0;
    for(uint f_idx=0; f_idx<active_features.size(); ++f_idx) {
        if(lambda[f_idx]==0) {
            DEBUG_OUT(1,"Deleting " << active_features[f_idx]->identifier() );
            delete active_features[f_idx];
        } else {
            new_lambda[new_f_idx]            = lambda[f_idx];
            new_active_features[new_f_idx]   = active_features[f_idx];
            DEBUG_OUT(1,"Added " << new_active_features[new_f_idx]->identifier() << " (new_idx = " << new_f_idx << ", old_idx = " << f_idx << ") to new active features");
            ++new_f_idx;
        }
    }

    // Swap new and old data
    lbfgs_free(lambda);
    lambda = new_lambda;
    active_features.swap(new_active_features);
    old_active_features_size = active_features.size();
}

void KMarkovCRF::check_lambda_size() {

    DEBUG_OUT(1, "Checking size of parameter vector");

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
