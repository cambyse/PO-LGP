/*
 * StateActionValueFunction.h
 *
 *  Created on: May 8, 2012
 *      Author: robert
 */

#ifndef STATEACTIONVALUEFUNCTION_H_
#define STATEACTIONVALUEFUNCTION_H_

#define DEBUG_STRING "StateActionValueFunction: "
#define DEBUG_LEVEL 1
#include "debug.h"

#include "TransitionGraph.h"

#include <float.h>
#include <vector>
using std::vector;

template <class State, class Action>
class StateActionValueFunction {

public:

	StateActionValueFunction(TransitionGraph<State,Action> * const t, const double& d ): transitionFunction(t), discount(d) {}

	virtual ~StateActionValueFunction() {}

	void init();
	void iterate();

	double get_min_value() { return min_state_value; }
	double get_max_value() { return max_state_value; }

	void set_discount(const double& d) { discount = d; }

private:

	TransitionGraph<State,Action> * transitionFunction;

	double discount, min_state_value, max_state_value;

	vector<State*> all_states;
	vector<double> tmp_state_values;
};

template <class State, class Action>
void StateActionValueFunction<State,Action>::init() {
	all_states = transitionFunction->get_all_states();
	tmp_state_values.clear();
	tmp_state_values.resize(all_states.size(),0);
	min_state_value = 0;
	max_state_value = 0;
	DEBUG_OUT(2,"States:");
	for(uint state_idx = 0; state_idx<all_states.size(); ++state_idx) {
	    all_states[state_idx]->state_action_value_state_model.available_actions = transitionFunction->get_available_actions(all_states[state_idx]);
	    all_states[state_idx]->state_action_value_state_model.adjacent_states = transitionFunction->get_adjacent_states(all_states[state_idx]);
	    all_states[state_idx]->state_action_value_state_model.action_values.clear();
	    all_states[state_idx]->state_action_value_state_model.action_values.resize(all_states[state_idx]->state_action_value_state_model.available_actions.size(),0);
	    all_states[state_idx]->state_action_value_state_model.state_value = 0;
	    DEBUG_OUT(2,"(" << all_states[state_idx]->grid_world_state_model.get_x() << "," << all_states[state_idx]->grid_world_state_model.get_y() << ")");
	}
    if(DEBUG_LEVEL>=2) {
        for(uint state_idx = 0; state_idx<all_states.size(); ++state_idx) {
            DEBUG_OUT(2,"In state " << state_idx <<
                    ": (" << all_states[state_idx]->grid_world_state_model.get_x() << "," << all_states[state_idx]->grid_world_state_model.get_y() << ")");
            vector<Action const *> avail_actions = all_states[state_idx]->state_action_value_state_model.available_actions;
            vector<State const *> adj_states = all_states[state_idx]->state_action_value_state_model.adjacent_states;
            for(uint idx=0; idx<avail_actions.size(); ++idx) {
                DEBUG_OUT(2,"Avail action number " << idx << ": " << avail_actions[idx]->get_action() );
            }
            for(uint idx=0; idx<adj_states.size(); ++idx) {
                DEBUG_OUT(2,"Adj state number " << idx <<
                        ": (" << adj_states[idx]->grid_world_state_model.get_x() << "," << adj_states[idx]->grid_world_state_model.get_y() << ")");
            }
        }
	}
}

template <class State, class Action>
void StateActionValueFunction<State,Action>::iterate() {

    min_state_value =  DBL_MAX;
    max_state_value = -DBL_MAX;
    State * source_state;
    State const * target_state;
    Action const * action;

    double single_action_value, action_value_sum, max_action_value;

    for(uint source_idx = 0; source_idx<all_states.size(); ++source_idx) // for all states
    {
        source_state = all_states[source_idx];
        vector<Action const *> & available_actions = source_state->state_action_value_state_model.available_actions;
        vector<State const *> & adjacent_states = source_state->state_action_value_state_model.adjacent_states;
        max_action_value = -DBL_MAX;
        for(uint action_idx = 0; action_idx<available_actions.size(); ++action_idx) // for all available actions
        {
            action = available_actions[action_idx];
            action_value_sum = 0;
            for(uint target_idx = 0; target_idx<adjacent_states.size(); ++target_idx) // for all adjacent states
            {
                target_state = adjacent_states[target_idx];
                single_action_value  = transitionFunction->get_transition_probability(source_state,action,target_state);
                single_action_value *= (target_state->state_action_value_state_model.state_value + target_state->state_action_value_state_model.state_reward);
                action_value_sum += single_action_value;
            }
            action_value_sum *= discount;
            source_state->state_action_value_state_model.action_values[action_idx] = action_value_sum;
            max_action_value = action_value_sum>max_action_value ? action_value_sum : max_action_value;
        }
        tmp_state_values[source_idx] = max_action_value;
        min_state_value = max_action_value<min_state_value ? max_action_value : min_state_value;
        max_state_value = max_action_value>max_state_value ? max_action_value : max_state_value;
    }

    // copy temporary values to states
    for(uint source_idx = 0; source_idx<all_states.size(); ++source_idx) // for all states
    {
        all_states[source_idx]->state_action_value_state_model.state_value = tmp_state_values[source_idx];
    }
}

#include "debug_exclude.h"

#endif /* STATEACTIONVALUEFUNCTION_H_ */
