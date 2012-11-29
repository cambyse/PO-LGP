/*
 * QIteration.h
 *
 *  Created on: Nov 28, 2012
 *      Author: robert
 */

#ifndef QITERATION_H_
#define QITERATION_H_

#include "Data.h"
#include <armadillo>

class QIteration
{

public:

    typedef Data::action_t      action_t;
    typedef Data::state_t       state_t;
    typedef Data::reward_t      reward_t;
    typedef Data::probability_t probability_t;
    typedef Data::input_data_t  input_data_t;

    QIteration();

    virtual ~QIteration();

    void set_transition_probability(state_t, action_t, state_t, probability_t) {}

    void set_reward_probability(input_data_t) {}
};

#endif /* QITERATION_H_ */
