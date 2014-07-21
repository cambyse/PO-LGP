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
    /** \brief Returns the transition probabilities for all possible
     * observation-reward pairs.
     *
     * The default implementation simply calls get_prediction() for all pairs
     * but derived class may implement this method much more efficiently. */
    virtual probability_map_t get_prediction_map(const_instance_ptr_t,
                                                 const action_ptr_t&) const;
};

#endif /* PREDICTOR_H_ */
