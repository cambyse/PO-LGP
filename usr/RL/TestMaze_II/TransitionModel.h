/*
 * TransitionModel.h
 *
 *  Created on: Oct 29, 2012
 *      Author: robert
 */

#ifndef TRANSITIONMODEL_H_
#define TRANSITIONMODEL_H_

#include <map>
#include <tuple>

template < class State, class Action >
class TransitionModel
{
public:
    TransitionModel(const int& x, const int& y): x_dim(x), y_dim(y) {}
    virtual ~TransitionModel() {}
    double get_transition_probability(const State& state_from, const Action& action, const State& state_to);
    void set_transition_probability(const State& state_from, const Action& action, const State& state_to, const double& p);

private:
    int x_dim, y_dim;
    std::map<std::tuple<State,Action,State>,double> probability_table;
};

template < class State, class Action >
double TransitionModel<State,Action>::get_transition_probability(const State& state_from, const Action& action, const State& state_to) {
    return probability_table[std::make_tuple(state_from,action,state_to)];
}

template < class State, class Action >
void TransitionModel<State,Action>::set_transition_probability(const State& state_from, const Action& action, const State& state_to, const double& p) {
    probability_table[std::make_tuple(state_from,action,state_to)] = p;
}

#endif /* TRANSITIONMODEL_H_ */
