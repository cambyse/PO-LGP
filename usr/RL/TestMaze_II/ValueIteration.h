/*
 * TransitionModel.h
 *
 *  Created on: Oct 29, 2012
 *      Author: robert
 */

#ifndef VALUEITERATION_H_
#define VALUEITERATION_H_

#include "Data.h"

#include <map>
#include <tuple>
#include <float.h>

class ValueIteration
{
public:
    typedef std::map<Data::state_t,double>                                          state_value_table_t;
    typedef std::map<std::tuple<Data::state_t,Data::action_t>,double>               state_action_value_table_t;
    typedef std::map<std::tuple<Data::state_t,Data::action_t,Data::state_t>,double> probability_table_t;
    typedef std::map<Data::state_t,Data::action_t>                                  state_action_table_t;

    ValueIteration(const double& disc = 0.9): discount(disc) {
        state_action_value_table = new state_action_value_table_t;
        old_state_action_value_table = new state_action_value_table_t;
    }
    virtual ~ValueIteration() {
        delete state_action_value_table;
        delete old_state_action_value_table;
    }
    double get_transition_probability(const Data::state_t& state_from, const Data::action_t& action, const Data::state_t& state_to);
    void set_transition_probability(const Data::state_t& state_from, const Data::action_t& action, const Data::state_t& state_to, const double& p);
    double get_expected_reward(const Data::state_t& state);
    void set_expected_reward(const Data::state_t& state, const double& r);
    void iterate();
    double get_value(const Data::state_t& state);

private:
    double discount;
    probability_table_t probability_table;
    state_value_table_t reward_table, state_value_table;
    state_action_value_table_t *state_action_value_table, *old_state_action_value_table;
    state_action_table_t state_action_table;
};

#endif /* VALUEITERATION_H_ */
