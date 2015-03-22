/*
 * StateActionValueState.h
 *
 *  Created on: May 8, 2012
 *      Author: robert
 */

#ifndef STATEACTIONVALUESTATEMODEL_H_
#define STATEACTIONVALUESTATEMODEL_H_

#include <vector>
using std::vector;

template <class State, class Action>
class StateActionValueStateModel {

public:

	StateActionValueStateModel(): state_reward(0), state_value(0) {}
	virtual ~StateActionValueStateModel(){}

	vector<Action const *> available_actions;
	vector<State const *> adjacent_states;
	vector<double> action_values;
	double state_reward, state_value;
};

#endif /* STATEACTIONVALUESTATEMODEL_H_ */
