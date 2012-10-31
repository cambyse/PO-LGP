/*
 * TransitionModel.h
 *
 *  Created on: Oct 29, 2012
 *      Author: robert
 */

#ifndef TRANSITIONMODEL_H_
#define VALUEITERATION_H_

#include <map>
#include <tuple>
#include <float.h>

#define DEBUG_LEVEL 0
#include "debug.h"

template < class State, class Action >
class ValueIteration
{
public:
    typedef std::map<State,double> state_value_table_t;
    typedef std::map<std::tuple<State,Action>,double> state_action_value_table_t;
    typedef std::map<std::tuple<State,Action,State>,double> probability_table_t;
    typedef std::map<State,Action> state_action_table_t;

    ValueIteration(const double& disc = 0.9): discount(disc) {
        state_action_value_table = new state_action_value_table_t;
        old_state_action_value_table = new state_action_value_table_t;
    }
    virtual ~ValueIteration() {
        delete state_action_value_table;
        delete old_state_action_value_table;
    }
    double get_transition_probability(const State& state_from, const Action& action, const State& state_to);
    void set_transition_probability(const State& state_from, const Action& action, const State& state_to, const double& p);
    double get_expected_reward(const State& state);
    void set_expected_reward(const State& state, const double& r);
    void iterate();
    double get_value(const State& state);

private:
    double discount;
    probability_table_t probability_table;
    state_value_table_t reward_table, state_value_table;
    state_action_value_table_t *state_action_value_table, *old_state_action_value_table;
    state_action_table_t state_action_table;
};

template < class State, class Action >
double ValueIteration<State,Action>::get_transition_probability(const State& state_from, const Action& action, const State& state_to) {
    return probability_table[std::make_tuple(state_from,action,state_to)];
}

template < class State, class Action >
void ValueIteration<State,Action>::set_transition_probability(const State& state_from, const Action& action, const State& state_to, const double& p) {
    probability_table[std::make_tuple(state_from,action,state_to)] = p;
    state_action_value_table->insert(std::make_pair(std::make_tuple(state_from,action),0));
    old_state_action_value_table->insert(std::make_pair(std::make_tuple(state_from,action),0));
    state_value_table.insert(std::make_pair(state_to,0));
}

template < class State, class Action >
double ValueIteration<State,Action>::get_expected_reward(const State& state) {
    return reward_table[state];
}

template < class State, class Action >
void ValueIteration<State,Action>::set_expected_reward(const State& state, const double& r) {
    reward_table[state] = r;
}

template < class State, class Action >
void ValueIteration<State,Action>::iterate() {
    // swap tables
    state_action_value_table_t * tmp_state_action_value_table = old_state_action_value_table;
    old_state_action_value_table = state_action_value_table;
    state_action_value_table = tmp_state_action_value_table;

    // clear state action values
    for(typename state_action_value_table_t::iterator it = state_action_value_table->begin(); it != state_action_value_table->end(); ++it) {
        (*it).second = 0;
    }
    // update state action values
    for(typename probability_table_t::iterator it = probability_table.begin(); it != probability_table.end(); ++it) {
        State state_from = std::get<0>(it->first);
        Action action = std::get<1>(it->first);
        State state_to = std::get<2>(it->first);
        double prob = it->second;
        typename state_value_table_t::iterator rewIt = reward_table.find(state_to);
        double reward = rewIt==reward_table.end() ? 0 : rewIt->second;

        (*state_action_value_table)[std::make_tuple(state_from,action)] += prob * (reward + discount*state_value_table[state_to]);
    }

    // clear state values
    for(typename state_value_table_t::iterator it = state_value_table.begin(); it != state_value_table.end(); ++it) {
        it->second = -DBL_MAX;
    }
    // update state values
    for(typename state_action_value_table_t::iterator stateActionValueIt = state_action_value_table->begin(); stateActionValueIt != state_action_value_table->end(); ++stateActionValueIt) {
        State state= std::get<0>(stateActionValueIt->first);
        Action action = std::get<1>(stateActionValueIt->first);
        typename state_value_table_t::iterator stateValueIt = state_value_table.find(state);
        if(stateValueIt==state_value_table.end()) {
            DEBUG_OUT(0,"Something is wrong: entry exists in state-action-value-table but not in state-value-table.");
            continue;
        }
        if(stateActionValueIt->second > stateValueIt->second) {
            stateValueIt->second = stateActionValueIt->second;
            state_action_table[state] = action;
        }
    }
}

template < class State, class Action >
double ValueIteration<State,Action>::get_value(const State& state) {
    return state_value_table[state];
}

#include "debug_exclude.h"

#endif /* VALUEITERATION_H_ */
