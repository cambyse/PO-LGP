#ifndef PREDICTIVEENVIRONMENT_H_
#define PREDICTIVEENVIRONMENT_H_

#include "Predictor.h"
#include "Environment.h"

class PredictiveEnvironment: public Predictor, public Environment {

public:

    DISAMBIGUATE_CONFIG_TYPEDEFS(Predictor);

    PredictiveEnvironment(action_ptr_t as, observation_ptr_t os, reward_ptr_t rs);

    virtual ~PredictiveEnvironment() override;

    /** \brief Perform a transition by executing an action. */
    virtual void perform_transition(const action_ptr_t & action) override;

    /** \brief Perform a transition by executing an action and return resulting
     * observation and reward by reference. */
    virtual void perform_transition(const action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r ) override;

    virtual const instance_ptr_t get_current_instance() const { return current_instance; }

protected:

    /**\brief Current instance defining the state of the environment.
     *
     * This instance is used to perform a transition based on the predicted
     * distribution defined by Predictor::get_prediction. */
    instance_ptr_t current_instance;

};

#endif /* PREDICTIVEENVIRONMENT_H_ */
