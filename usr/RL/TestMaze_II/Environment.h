#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "SpaceManager.h"

#include "Feature.h"
#include "FeatureLearner.h"

class Environment: public virtual SpaceManager {

public:

    typedef Feature::const_feature_ptr_t f_ptr_t;

    Environment() = default;
    virtual ~Environment() = default;

    /** \brief Perform a transition by executing an action. */
    virtual void perform_transition(const action_ptr_t & action) = 0;

    /** \brief Perform a transition by executing an action and return resulting
     * observation and reward by reference. */
    virtual void perform_transition(const action_ptr_t & a,
                                    observation_ptr_t & o,
                                    reward_ptr_t & r ) = 0;

    virtual void get_features(std::vector<f_ptr_t> & basis_features,
                              FeatureLearner::LEARNER_TYPE type) const = 0;
};

#endif /* ENVIRONMENT_H_ */
