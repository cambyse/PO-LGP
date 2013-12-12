#ifndef PREDICTIVEENVIRONMENT_H_
#define PREDICTIVEENVIRONMENT_H_

#include "Config.h"
#include "AbstractAction.h"
#include "AbstractObservation.h"
#include "AbstractReward.h"
#include "Instance.h"
#include "Feature.h"
#include "FeatureLearner.h"

class PredictiveEnvironment {
public:

    USE_CONFIG_TYPEDEFS;
    typedef Feature::const_feature_ptr_t f_ptr_t;

    PredictiveEnvironment() = default;
    virtual ~PredictiveEnvironment() = default;

    /** \brief Perform a transition by executing an action. */
    virtual void perform_transition(const action_ptr_t & action) = 0;

    /** \brief Perform a transition by executing an action and return resulting
     * observation and reward by reference. */
    virtual void perform_transition(const action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r ) = 0;

    /** \brief Returns the transition probability. */
    virtual probability_t get_prediction(const instance_t*, const action_ptr_t&, const observation_ptr_t&, const reward_ptr_t&) const = 0;

    virtual void get_features(std::vector<f_ptr_t> & basis_features, FeatureLearner::LEARNER_TYPE type) const = 0;

    virtual void get_spaces(action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r) const = 0;
};

#endif /* PREDICTIVEENVIRONMENT_H_ */
