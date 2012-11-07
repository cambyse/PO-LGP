
#include "KMarkovCRF.h"

#define DEBUG_STRING ""
#define DEBUG_LEVEL 2
#include "debug.h"


const char* KMarkovCRF::ActionFeature::action_strings[5] = { " STAY", "   UP", " DOWN", " LEFT", "RIGHT" };

template < class Data, class DataPredict>
int KMarkovCRF::Feature<Data,DataPredict>::field_width[2] = {0,0};

KMarkovCRF::KMarkovCRF(
        const int& ,
        const int& ,
        const int& k_,
        const int& x,
        const int& y,
        const int& a_n):
        k(k_), x_dim(x), y_dim(y), action_n(a_n) {

    number_of_features = (k+1)*(x_dim*y_dim + action_n + 2);
    number_of_features += 1; // todo for the combined feature below...
    lambda = lbfgs_malloc(number_of_features);
    if (lambda == nullptr) {
        printf("ERROR: Failed to allocate a memory block for variables.\n");
        lbfgs_free(lambda);
        return;
    }

    int linear_idx = 0;
    feature_t  *reward_1_0, *state_3_2;
    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx<=k; ++k_idx) {
        // actions
        for(action_t action=0; action<action_n; ++action) {
            lambda[linear_idx] = 0;
            features.push_back(std::unique_ptr<feature_t>(new ActionFeature(action,k_idx)));
            parameter_indices.push_back(linear_idx);
            ++linear_idx;
        }
        // states
        for(state_t state=0; state<x_dim*y_dim; ++state) {
            lambda[linear_idx] = 0;
            features.push_back(std::unique_ptr<feature_t>(new StateFeature(state,k_idx)));
            parameter_indices.push_back(linear_idx);
            ++linear_idx;
            if(state==3 && k_idx==2) {
                state_3_2 = &(*(features.back()));
                DEBUG_OUT(0,"Identified " << state_3_2->identifier() );
            }
        }
        // reward
        for(reward_t reward = 0.0; reward<=1.0; ++reward) {
            lambda[linear_idx] = 0;
            features.push_back(std::unique_ptr<feature_t>(new RewardFeature(reward,k_idx)));
            parameter_indices.push_back(linear_idx);
            ++linear_idx;
            if(reward==1.0 && k_idx==0) {
                reward_1_0 = &((*features.back()));
                DEBUG_OUT(0,"Identified " << reward_1_0->identifier() );
            }
        }
    }

    lambda[linear_idx] = 0;
    features.push_back(std::unique_ptr<feature_t>(new AndFeature((*reward_1_0),(*state_3_2))));
    parameter_indices.push_back(linear_idx);
    DEBUG_OUT(0,"Added " << features.back()->identifier() );
    ++linear_idx;

    if(linear_idx!=number_of_features) {
        DEBUG_OUT(0,"Error number of features and number of parameters do not match.");
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
        for(int f_idx=0; f_idx<number_of_features; ++f_idx) { // sum over features
            sumFNN[data_idx] += x[parameter_indices[f_idx]]*features[f_idx]->evaluate(episode_iterator);
        }

        // calculate sumExp(x(n))
        for(state_t state=0; state<x_dim*y_dim; ++state) { // sum over y' (states and rewards)
            for(reward_t reward=0.0; reward<=1.0; ++reward) {
                data_point_t data_predict = std::make_tuple(action_t(),state,reward); // y'

                // calculate sumF(x(n),y')
                double sumFN = 0;
                for(int f_idx=0; f_idx<number_of_features; ++f_idx) { // sum over features
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
    fx *= -1;
    for(int i=0; i<n; i++) {
        g[i] *= -1;
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
        const lbfgsfloatval_t /*xnorm*/,
        const lbfgsfloatval_t /*gnorm*/,
        const lbfgsfloatval_t /*step*/,
        int /*n*/,
        int k,
        int /*ls*/
) {
    DEBUG_OUT(0,"Iteration " << k << " (fx = " << fx << "):");
    for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
        int l_idx = parameter_indices[f_idx];
        DEBUG_OUT(1, "    " <<
                features[f_idx]->identifier() <<
                " --> t[" <<
                l_idx << "] = " <<
                x[l_idx]);
    }
    DEBUG_OUT(1,"");

    return 0;
}

void KMarkovCRF::optimize_model() {
    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    // param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(number_of_features, lambda, &fx, static_evaluate_model, static_progress_model, this, &param);

    // Report the result.
    DEBUG_OUT(0, "L-BFGS optimization terminated with status code = " << ret);
    DEBUG_OUT(0,"mean likelihood = " << exp(-fx/(episode_data.size()-k)) );
    for(uint f_idx=0; f_idx<features.size(); ++f_idx) {
        int l_idx = parameter_indices[f_idx];
        DEBUG_OUT(1, "    " <<
                features[f_idx]->identifier() <<
                " --> t[" <<
                l_idx << "] = " <<
                lambda[l_idx]);
    }
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
    lbfgsfloatval_t * x = lbfgs_malloc(number_of_features);
    lbfgsfloatval_t * dx = lbfgs_malloc(number_of_features);
    lbfgsfloatval_t * grad = lbfgs_malloc(number_of_features);
    lbfgsfloatval_t * grad_dummy = lbfgs_malloc(number_of_features);

    // remember deviations
    lbfgsfloatval_t relative_deviation = 0;

    DEBUG_OUT(0,"Checking first derivative (#samples="<<number_of_samples<<", range=+/-"<<range<<", max_var="<<max_variation<<", max_rel_dev="<<max_relative_deviation);
    for(int count=0; count<number_of_samples; ++count) {

        // set test point
        for(int x_idx=0; x_idx<number_of_features; ++x_idx) {
            x[x_idx] = range * (2*drand48()-1);
            dx[x_idx] = (2*drand48()-1)*max_variation;
        }

        lbfgsfloatval_t fx = evaluate_model(x,grad,number_of_features);
        DEBUG_OUT(1, "fx = " << fx );
        for(int x_idx=0; x_idx<number_of_features; ++x_idx) {

            // go in positive direction
            x[x_idx] += dx[x_idx]/2.;
            lbfgsfloatval_t fx_plus = evaluate_model(x,grad_dummy,number_of_features);
            // go in negative direction
            x[x_idx] -= dx[x_idx];
            lbfgsfloatval_t fx_minus = evaluate_model(x,grad_dummy,number_of_features);
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
