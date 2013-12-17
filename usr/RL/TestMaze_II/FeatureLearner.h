#ifndef FEATURELEARNER_H_
#define FEATURELEARNER_H_

#include "AbstractAction.h"
#include "AbstractObservation.h"
#include "AbstractReward.h"

class PredictiveEnvironment;

class FeatureLearner {

public:

    // typedefs
    typedef AbstractAction::ptr_t action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t reward_ptr_t;
    enum class LEARNER_TYPE { FULL_PREDICTIVE, HISTORY_ONLY, HISTORY_AND_ACTION };

    // constructor/destructor
    FeatureLearner(const LEARNER_TYPE lt): learner_type(lt) {}
    virtual ~FeatureLearner() = default;

    /** \brief Initialize action, observation, and reward spaces. */
    virtual void initialize_spaces(const PredictiveEnvironment & environment);

private:

    const LEARNER_TYPE learner_type;      ///< Characterize the different types of feature learners.
    action_ptr_t       action_space;      ///< The action space that is used.
    observation_ptr_t  observation_space; ///< The observation space that is used.
    reward_ptr_t       reward_space;      ///< The reward space that is used.
};

#endif /* FEATURELEARNER_H_ */
