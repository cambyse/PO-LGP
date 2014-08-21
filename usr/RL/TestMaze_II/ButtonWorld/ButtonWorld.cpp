#include "ButtonWorld.h"

#include <algorithm> // std::next_permutation, std::sort
#include <gsl/gsl_rng.h> // random generator
#include <gsl/gsl_randist.h> // Dirichlet distribution

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using std::vector;

static const double button_size = 0.9;
static const QPen button_pen_OFF(QColor(0,0,0),0.03,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
static const QPen button_pen_ON(QColor(0,0,0),0.1,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
static const QBrush reward_brush_NONE(QColor(200, 200, 200));
static const QBrush reward_brush_PLUS(QColor(0, 200, 0));
static const QBrush reward_brush_MINUS(QColor(200, 0, 0));
static const QPen reward_pen(QColor(0,0,0),0.03,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
static const double text_scale = 0.008;
static const QFont  text_font = QFont("Helvetica [Cronyx]", 12);

ButtonWorld::ButtonWorld(int s, std::vector<probability_t> p):
    PredictiveEnvironment(action_ptr_t(), observation_ptr_t(), reward_ptr_t()),
    size(s),
    prob_array(p)
{
    // set probabilities
    if(prob_array.size() == 0) {
        DEBUG_OUT(2, "Empty probabilities, initializing randomly");
        prob_array.resize(size);
        for(auto& elem : prob_array) {
            elem = drand48();
            DEBUG_OUT(2, elem);
        }
    }
}

void ButtonWorld::get_features(f_set_t & basis_features,
                               FeatureLearner::LEARNER_TYPE type) const {

    // clear first
    basis_features.clear();

    // add features
    int k = 1;
    for(int k_idx = 0; k_idx>=-k; --k_idx) {
        if((type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_ONLY && k_idx<0) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION)) {
            // actions
            for(action_ptr_t action : action_space) {
                f_ptr_t action_feature = ActionFeature::create(action,k_idx);
                DEBUG_OUT(2,"Adding feature: " << *action_feature);
                basis_features.insert(action_feature);
                if(use_factored_action_features) {
                    construct_factored_action_features(basis_features, action_space, k_idx);
                }
            }
        }
        if((type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_ONLY && k_idx<0) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION && k_idx<0)) {
            // observations
            for(observation_ptr_t observation : observation_space) {
                f_ptr_t observation_feature = ObservationFeature::create(observation,k_idx);
                DEBUG_OUT(2,"Adding feature: " << *observation_feature);
                basis_features.insert(observation_feature);
            }
        }
        if((type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE && k_idx==0) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_ONLY && k_idx<0) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION && k_idx<0)) {
            // reward
            for(reward_ptr_t reward : reward_space) {
                f_ptr_t reward_feature = RewardFeature::create(reward,k_idx);
                DEBUG_OUT(2,"Adding feature: " << *reward_feature);
                basis_features.insert(reward_feature);
            }
        }
    }
    if(type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION) {
        // also add a unit feature
        f_ptr_t const_feature = ConstFeature::create(1);
        DEBUG_OUT(2,"Adding feature: " << *const_feature);
        basis_features.insert(const_feature);
    }
}

void ButtonWorld::construct_factored_action_features(f_set_t & basis_features,
                                                     action_ptr_t action,
                                                     int delay) {
    auto button_action = action.get_derived<ButtonAction>();
    for(int idx : util::Range(button_action->get_array().size())) {
        {
            f_ptr_t action_feature = ButtonActionFeature::create(idx,delay, true);
            DEBUG_OUT(2,"Adding feature: " << *action_feature);
            basis_features.insert(action_feature);
        }
        {
            f_ptr_t action_feature = ButtonActionFeature::create(idx,delay, false);
            DEBUG_OUT(2,"Adding feature: " << *action_feature);
            basis_features.insert(action_feature);
        }
    }
}

std::vector<ButtonWorld::probability_t> ButtonWorld::probs_from_beta(const int& s,
                                                                     const double& alpha) {
    // set up random generator
    gsl_rng * rand_gen = gsl_rng_alloc(gsl_rng_default);
    gsl_rng_set(rand_gen, time(nullptr));
    // draw
    vector<double> probs;
    repeat(s) {
        probs.push_back(gsl_ran_beta(rand_gen, alpha, alpha));
    }
    // clean up
    gsl_rng_free(rand_gen);
    // return
    return probs;
}
