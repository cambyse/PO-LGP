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
    typedef reward_t            value_t;
    typedef Data::probability_t probability_t;
    typedef Data::k_mdp_state_t k_mdp_state_t;

    typedef std::vector<arma::SpMat<probability_t> > sparse_probability_cube_t;
    typedef std::vector<arma::SpMat<reward_t> >      sparse_reward_cube_t;
    typedef arma::Mat<value_t>                       q_table_t;
    typedef arma::Col<value_t>                       value_vector_t;

    QIteration();

    virtual ~QIteration();

    void set_transition_probability(state_t, action_t, state_t, probability_t);
    void set_transition_probability(k_mdp_state_t, action_t, state_t, probability_t);
    void set_expected_reward(k_mdp_state_t, action_t, state_t, reward_t);

protected:

};

#endif /* QITERATION_H_ */
