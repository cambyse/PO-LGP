#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include "SpaceManager.h"

class Predictor: public virtual SpaceManager {
public:

    Predictor() = default;
    virtual ~Predictor() = default;

    /** \brief Returns the transition probability. */
    virtual probability_t get_prediction(const_instance_ptr_t,
                                         const action_ptr_t&,
                                         const observation_ptr_t&,
                                         const reward_ptr_t&) const = 0;
};

#endif /* PREDICTOR_H_ */
