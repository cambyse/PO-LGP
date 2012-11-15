
#include "KMarkovCRF.h"

#define DEBUG_STRING ""
#define DEBUG_LEVEL 2
#include "debug.h"

KMarkovCRF::KMarkovCRF( const int& k_ ): k(k_) {

    data_features_n = k*(Data::action_n + Data::state_n + Data::reward_n) + Data::action_n;
    predict_features_n = Data::state_n + Data::reward_n;
    combined_features_n = Data::state_n*Data::action_n*Data::state_n + (k+1)*Data::state_n*Data::reward_n;

    lambda = lbfgs_malloc(data_features_n+predict_features_n+combined_features_n);
    if (lambda == nullptr) {
        printf("ERROR: Failed to allocate a memory block for variables.\n");
        lbfgs_free(lambda);
        return;
    }

    //----------------------------------------//
    // Constructing basic indicator features  //
    //----------------------------------------//
    int linear_idx = 0;
    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-k; --k_idx) {
        // actions
        for(action_t action=0; action<Data::action_n; ++action) {
            ActionFeature * action_feature = new ActionFeature(action,k_idx);
            lambda[linear_idx] = 0;
            parameter_indices.push_back(linear_idx);
            features.push_back(std::unique_ptr<Feature>(action_feature));
            data_features.push_back(action_feature);
            ++linear_idx;
        }
        // states
        for(state_t state=0; state<Data::state_n; ++state) {
            StateFeature * state_feature = new StateFeature(state,k_idx);
            lambda[linear_idx] = 0;
            parameter_indices.push_back(linear_idx);
            features.push_back(std::unique_ptr<Feature>(state_feature));
            if(k_idx==0) {
                predict_features.push_back(state_feature);
            } else {
                data_features.push_back(state_feature);
            }
            ++linear_idx;
        }
        // reward
        for(reward_t reward = Data::min_reward; reward<=Data::max_reward; reward+=Data::reward_increment) {
            RewardFeature * reward_feature = new RewardFeature(reward,k_idx);
            lambda[linear_idx] = 0;
            parameter_indices.push_back(linear_idx);
            features.push_back(std::unique_ptr<Feature>(reward_feature));
            if(k_idx==0) {
                predict_features.push_back(reward_feature);
            } else {
                data_features.push_back(reward_feature);
            }
            ++linear_idx;
        }
    }
    // check number of features
    if((int)features.size()!=data_features_n+predict_features_n) {
        DEBUG_OUT(0,"Error: Number of data and prediction features not as expected!");
        DEBUG_OUT(0,"    #features            = " << features.size());
        DEBUG_OUT(0,"    #data features       = " << data_features.size()    << " (" << data_features_n    << ")" );
        DEBUG_OUT(0,"    #prediction features = " << predict_features.size() << " (" << predict_features_n << ")" );
    }

    //----------------------------------//
    // Constructing MDP state features  //
    //----------------------------------//
//    for(state_t state_from=0; state_from<Data::state_n; ++state_from) {
//        for(action_t action=0; action<Data::action_n; ++action) {
//            for(state_t state_to=0; state_to<Data::state_n; ++state_to) {
//                StateFeature * state_from_feature = new StateFeature(state_from,-1);
//                ActionFeature * action_feature = new ActionFeature(action,0);
//                StateFeature * state_to_feature = new StateFeature(state_to,0);
//                AndFeature * and_feature_state = new AndFeature((*state_from_feature),(*action_feature),(*state_to_feature));
//                lambda[linear_idx] = 0;
//                parameter_indices.push_back(linear_idx);
//                features.push_back(std::unique_ptr<Feature>(and_feature_state));
//                combined_features.push_back(and_feature_state);
//                subfeatures.push_back(std::unique_ptr<Feature>(state_from_feature));
//                subfeatures.push_back(std::unique_ptr<Feature>(action_feature));
//                subfeatures.push_back(std::unique_ptr<Feature>(state_to_feature));
//                ++linear_idx;
//            }
//        }
//    }

    //-----------------------------------//
    // Constructing MDP reward features  //
    //-----------------------------------//
//    RewardFeature * reward_yes_feature = new RewardFeature(1.0,0);
//    RewardFeature * reward_no_feature = new RewardFeature(0.0,0);
//    subfeatures.push_back(std::unique_ptr<Feature>(reward_yes_feature));
//    subfeatures.push_back(std::unique_ptr<Feature>(reward_no_feature));
//    for(int k_idx=0; k_idx>=-k; --k_idx) {
//        for(state_t state=0; state<Data::state_n; ++state) {
//            StateFeature * state_feature = new StateFeature(state,k_idx);
//            subfeatures.push_back(std::unique_ptr<Feature>(state_feature));
//
//            AndFeature * and_feature_reward_yes = new AndFeature((*state_feature),(*reward_yes_feature));
//            lambda[linear_idx] = 0;
//            parameter_indices.push_back(linear_idx);
//            features.push_back(std::unique_ptr<Feature>(and_feature_reward_yes));
//            combined_features.push_back(and_feature_reward_yes);
//            ++linear_idx;
//
//            AndFeature * and_feature_reward_no = new AndFeature((*state_feature),(*reward_no_feature));
//            lambda[linear_idx] = 0;
//            parameter_indices.push_back(linear_idx);
//            features.push_back(std::unique_ptr<Feature>(and_feature_reward_no));
//            combined_features.push_back(and_feature_reward_no);
//            ++linear_idx;
//        }
//    }

    // check number of features
    if((int)combined_features.size()!=combined_features_n) {
        DEBUG_OUT(0,"Error: Number of combined features not as expected!");
        DEBUG_OUT(0,"    #combined features = " << combined_features.size() << " (" << combined_features_n << ")" );
    }

    // create output indicator features
    for(state_t state=0; state<Data::state_n; ++state) {
        StateFeature * state_feature = new StateFeature(state,0);
        subfeatures.push_back(std::unique_ptr<Feature>(state_feature));
        for(reward_t reward=Data::min_reward; reward<=Data::max_reward; reward+=Data::reward_increment) {
            RewardFeature * reward_feature = new RewardFeature(reward,0);
            subfeatures.push_back(std::unique_ptr<Feature>(reward_feature));
            AndFeature * and_feature = new AndFeature((*state_feature),(*reward_feature));
            output_features.push_back(std::unique_ptr<Feature>(and_feature));
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

    std::vector<double> sumFNN(number_of_data_points,0.0);
    std::vector<double> sumExpN(number_of_data_points,0.0);
    std::vector<std::vector<double> > sumFExpNF(number_of_data_points,std::vector<double>(n,0.0));
    int data_idx = 0;
    for(const_episode_iterator_t episode_iterator=episode_data.begin()+k;
            episode_iterator!=episode_data.end();
            ++episode_iterator) {

        // calculate sumF(x(n),y(n))
        for(uint f_idx=0; f_idx<features.size(); ++f_idx) { // sum over features
            sumFNN[data_idx] += x[parameter_indices[f_idx]]*features[f_idx]->evaluate(episode_iterator);
        }

        // calculate sumExp(x(n))
        for(state_t state=0; state<Data::state_n; ++state) { // sum over y' (states and rewards)
            for(reward_t reward=Data::min_reward; reward<=Data::max_reward; reward+=Data::reward_increment) {
                data_point_t data_predict = std::make_tuple(action_t(),state,reward); // y'

                // calculate sumF(x(n),y')
                double sumFN = 0;
                for(uint f_idx=0; f_idx<features.size(); ++f_idx) { // sum over features
                    sumFN += x[parameter_indices[f_idx]]*features[f_idx]->evaluate(episode_iterator,data_predict);
                }

                // increment sumExp(x(n))
                sumExpN[data_idx] += exp( sumFN );

                // increment sumFExp(x(n),F)
                for(int lambda_idx=0; lambda_idx<n; ++lambda_idx) { // for all parameters/gradient components
                    // in case of parameter binding additionally sum over all features belonging to this parameter
                    if(parameter_indices[lambda_idx]!=lambda_idx) {
                        DEBUG_OUT(0,"Not the correct feature for this parameter, check parameter binding.");
                    } else {
                        sumFExpNF[data_idx][lambda_idx] += features[lambda_idx]->evaluate(episode_iterator,data_predict) * exp( sumFN );
                    }
                }

            }
        }

        // increment fx
        fx += sumFNN[data_idx] - log( sumExpN[data_idx] );

        // increment gradient
        for(int lambda_idx=0; lambda_idx<n; ++lambda_idx) { // for all parameters/gradient components
            g[lambda_idx] -= sumFExpNF[data_idx][lambda_idx]/sumExpN[data_idx];

            // in case of parameter binding additionally sum over all features belonging to this parameter
            if(parameter_indices[lambda_idx]!=lambda_idx) {
                DEBUG_OUT(0,"Not the correct feature for this parameter, check parameter binding.");
            } else {
                g[lambda_idx] += features[lambda_idx]->evaluate(episode_iterator);
            }
        }

        ++data_idx;
    }

    // use NEGATIVE log likelihood (blfgs minimizes the objective)
    // also use mean value per data point
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
    for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
        int l_idx = parameter_indices[f_idx];
        DEBUG_OUT(1, "    " <<
                features[f_idx]->identifier() <<
                " --> t[" <<
                l_idx << "] = " <<
                x[l_idx]);
    }
    DEBUG_OUT(0,"Iteration " << k << " (fx = " << fx << ", xnorm = " << xnorm << ", p = " << exp(-fx) << "):");
    DEBUG_OUT(1,"");

    return 0;
}

void KMarkovCRF::optimize_model(lbfgsfloatval_t l1) {
    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.orthantwise_c = l1;
    param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(features.size(), lambda, &fx, static_evaluate_model, static_progress_model, this, &param);

    // Report the result.
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx) );
    for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
        int l_idx = parameter_indices[f_idx];
        DEBUG_OUT(1, "    " <<
                features[f_idx]->identifier() <<
                " --> t[" <<
                l_idx << "] = " <<
                lambda[l_idx]);
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
    lbfgsfloatval_t * x = lbfgs_malloc(features.size());
    lbfgsfloatval_t * dx = lbfgs_malloc(features.size());
    lbfgsfloatval_t * grad = lbfgs_malloc(features.size());
    lbfgsfloatval_t * grad_dummy = lbfgs_malloc(features.size());

    // remember deviations
    lbfgsfloatval_t relative_deviation = 0;

    DEBUG_OUT(0,"Checking first derivative (#samples="<<number_of_samples<<", range=+/-"<<range<<", max_var="<<max_variation<<", max_rel_dev="<<max_relative_deviation);
    for(int count=0; count<number_of_samples; ++count) {

        // set test point
        for(uint x_idx=0; x_idx<features.size(); ++x_idx) {
            x[x_idx] = range * (2*drand48()-1);
            dx[x_idx] = (2*drand48()-1)*max_variation;
        }

        lbfgsfloatval_t fx = evaluate_model(x,grad,features.size());
        DEBUG_OUT(1, "fx = " << fx );
        for(uint x_idx=0; x_idx<features.size(); ++x_idx) {

            // go in positive direction
            x[x_idx] += dx[x_idx]/2.;
            lbfgsfloatval_t fx_plus = evaluate_model(x,grad_dummy,features.size());
            // go in negative direction
            x[x_idx] -= dx[x_idx];
            lbfgsfloatval_t fx_minus = evaluate_model(x,grad_dummy,features.size());
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
    for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
        DEBUG_OUT(0, "    " << features[f_idx]->identifier() << " = " << features[f_idx]->evaluate(episode_data.end()-1) );
    }
}

void KMarkovCRF::rank_pair_features() {
    int number_of_data_points = episode_data.size()-k;
    if(number_of_data_points<=0) {
        DEBUG_OUT(0,"Not enough data to evaluate model.");
        return;
    }

    // construct pair features
    DEBUG_OUT(0,"Constructing pair features");
    std::vector<std::unique_ptr<Feature> > pair_features;
    for(uint f1_idx=0; f1_idx<features.size(); ++f1_idx) {
        for(uint f2_idx=0; f2_idx<features.size(); ++f2_idx) {
            if(f1_idx!=f2_idx) {
                AndFeature * and_feature = new AndFeature((*features[f1_idx]),(*features[f2_idx]));
                pair_features.push_back(std::unique_ptr<Feature>(and_feature));
                DEBUG_OUT(1,"    added " << and_feature->identifier() );
            }
        }
    }
    int pair_feature_n = pair_features.size();
    int output_feature_n = output_features.size();

    // determine relative frequencies
    std::vector<int> pair_feature_counts(pair_feature_n,0);
    std::vector<int> output_feature_counts(output_feature_n,0);
    std::vector<int> joint_counts(pair_feature_n*output_feature_n,0);
    DEBUG_OUT(0,"Determining relative frequencies");
    for(const_episode_iterator_t episode_iterator=episode_data.begin()+k;
                episode_iterator!=episode_data.end();
                ++episode_iterator) {
        for(int output_feature_idx=0; output_feature_idx<output_feature_n; ++output_feature_idx) { // all output features
            if(output_features[output_feature_idx]->evaluate(episode_iterator)==1) {
                ++output_feature_counts[output_feature_idx];
            }
        }
        for(int pair_feature_idx=0; pair_feature_idx<pair_feature_n; ++pair_feature_idx) { // all pair features
            if(pair_features[pair_feature_idx]->evaluate(episode_iterator)==1) {
                ++pair_feature_counts[pair_feature_idx];
            }
        }
        for(int pair_feature_idx=0; pair_feature_idx<pair_feature_n; ++pair_feature_idx) { // all pair features
            for(int output_feature_idx=0; output_feature_idx<output_feature_n; ++output_feature_idx) { // all output features
                if(pair_features[pair_feature_idx]->evaluate(episode_iterator)==1 &&
                        output_features[output_feature_idx]->evaluate(episode_iterator)==1) {
                    ++joint_counts[pair_feature_idx + pair_feature_n*output_feature_idx];
                }
            }
        }
    }

    // print counts
    DEBUG_OUT(0,"Relative counts:");
    DEBUG_OUT(0,"    output feature counts");
    for(int output_feature_idx=0; output_feature_idx<output_feature_n; ++output_feature_idx) { // all output features
        DEBUG_OUT(0, "        " <<
                output_features[output_feature_idx]->identifier() << ": " <<
                output_feature_counts[output_feature_idx] << " (" <<
                (double)output_feature_counts[output_feature_idx]/number_of_data_points << ")"
        );
    }
    DEBUG_OUT(0,"    pair feature counts");
    for(int pair_feature_idx=0; pair_feature_idx<pair_feature_n; ++pair_feature_idx) { // all pair features
        DEBUG_OUT(0, "        " <<
                pair_features[pair_feature_idx]->identifier() << ": " <<
                pair_feature_counts[pair_feature_idx] << " (" <<
                (double)pair_feature_counts[pair_feature_idx]/number_of_data_points << ")"
        );
    }
    DEBUG_OUT(0,"    joint counts");
    for(int output_feature_idx=0; output_feature_idx<output_feature_n; ++output_feature_idx) { // all output features
        for(int pair_feature_idx=0; pair_feature_idx<pair_feature_n; ++pair_feature_idx) { // all pair features
            DEBUG_OUT(0, "        [" <<
                    output_features[output_feature_idx]->identifier() << "," <<
                    pair_features[pair_feature_idx]->identifier() << "]: " <<
                    joint_counts[pair_feature_idx + pair_feature_n*output_feature_idx] << " (" <<
                    (double)joint_counts[pair_feature_idx + pair_feature_n*output_feature_idx]/number_of_data_points << ")"
            );
        }
    }

    // determine rank
    std::vector<double> pair_feature_rank(pair_feature_n,0);
    for(int pair_feature_idx=0; pair_feature_idx<pair_feature_n; ++pair_feature_idx) { // all pair features
        for(int output_feature_idx=0; output_feature_idx<output_feature_n; ++output_feature_idx) { // all output features
            if(joint_counts[pair_feature_idx + pair_feature_n*output_feature_idx] == 0) continue;
            if(pair_feature_counts[pair_feature_idx]                              == 0) continue;
            if(output_feature_counts[output_feature_idx]                          == 0) continue;
            double frac = (double)joint_counts[pair_feature_idx + pair_feature_n*output_feature_idx]/number_of_data_points;
            frac /= (double)pair_feature_counts[pair_feature_idx]/number_of_data_points;
            frac /= (double)output_feature_counts[output_feature_idx]/number_of_data_points;
            pair_feature_rank[pair_feature_idx] += (double)joint_counts[pair_feature_idx + pair_feature_n*output_feature_idx]/number_of_data_points * log(frac);
//            DEBUG_OUT(0,"====Increment==== frac=" << frac <<
//                    ", log(frac)=" << log(frac) <<
//                    ", rank=" << pair_feature_rank[pair_feature_idx] <<
//                    ", joint/N=" << (double)joint_counts[pair_feature_idx + pair_feature_n*output_feature_idx]/number_of_data_points
//            );
        }
    }

    // print rank
    DEBUG_OUT(0,"Ranking features:");
    for(int f_idx=0; f_idx<pair_feature_n; ++f_idx) {
//        if(pair_feature_rank[f_idx]!=0) {
            DEBUG_OUT(0, "    " << pair_features[f_idx]->identifier() << ": " << pair_feature_rank[f_idx]);
//        }
    }

}
