#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include "Config.h"
#include "AbstractAction.h"
#include "AbstractObservation.h"
#include "AbstractReward.h"
#include "Instance.h"

class Predictor {
public:

    USE_CONFIG_TYPEDEFS;

    Predictor() = default;
    virtual ~Predictor() = default;

    /** \brief Returns the transition probability. */
    virtual probability_t get_prediction(const instance_t*, const action_ptr_t&, const observation_ptr_t&, const reward_ptr_t&) const = 0;
};

#endif /* PREDICTOR_H_ */
