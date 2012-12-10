/*
 * QIteration.h
 *
 *  Created on: Nov 28, 2012
 *      Author: robert
 */

#ifndef QITERATION_H_
#define QITERATION_H_

#include "Data.h"
#include <vector>

class QIteration
{

public:

    typedef Data::action_t      action_t;
    typedef Data::state_t       state_t;
    typedef Data::reward_t      reward_t;
    typedef reward_t            value_t;
    typedef Data::probability_t probability_t;
    typedef Data::k_mdp_state_t k_mdp_state_t;
    typedef unsigned long       idx_t;

    QIteration(const double& d = 0.9);

    virtual ~QIteration();

    void clear();

    void set_prediction(k_mdp_state_t,action_t,state_t,reward_t,probability_t);

    void set_discount(const double& d);
    double get_discount();

    value_t iterate();

    action_t optimal_action(const k_mdp_state_t&);

protected:

    double discount;
    std::vector<probability_t> prediction;
    std::vector<value_t>       state_value;
    std::vector<value_t>       state_action_value;

};

#endif /* QITERATION_H_ */
