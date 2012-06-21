/*
 * TransitionGraph.h
 *
 *  Created on: May 1, 2012
 *      Author: robert
 */

#ifndef TRANSITIONGRAPH_H_
#define TRANSITIONGRAPH_H_

#include "GraphModel.h"

#define DEBUG_STRING "TransitionGraph: "
#define DEBUG_LEVEL 1
#include "debug.h"

#include <vector>
using std::vector;

// State template should be derived from type NodeStateModel
template <class State, class Action>
class TransitionGraph {

public:

	TransitionGraph();
	virtual ~TransitionGraph() {}

	void test_function() {}

	State add_state();
	void add_state(State * s);

	vector<State *> get_all_states();
	vector<State const *> get_adjacent_states(const State * s);
	vector<Action const *> get_available_actions(const State * s);

	void add_transition(const State * s1, const Action * a, const State * s2, const double& probability, const bool& enabled = true, const bool& normalize = true);
	void remove_all_transitions();

	void normalize_transitions(const State * s);
	void normalize_all_transitions();

	void enable_transitions(const State * state_from, const State * state_to, const bool& normalize = true);
	void enable_transitions(const State * state_from, const State * state_to, const Action * action, const bool& normalize = true);
	void enable_bidirectional_transitions(const State * state_from, const State * state_to, const bool& normalize = true);
	void enable_bidirectional_transitions(const State * state_from, const State * state_to, const Action * action, const bool& normalize = true);
	void disable_transitions(const State * state_from, const State * state_to, const bool& normalize = true);
	void disable_transitions(const State * state_from, const State * state_to, const Action * action, const bool& normalize = true);
	void disable_bidirectional_transitions(const State * state_from, const State * state_to, const bool& normalize = true);
	void disable_bidirectional_transitions(const State * state_from, const State * state_to, const Action * action, const bool& normalize = true);

	virtual State * get_transition(State * const state_from, const Action * action) const;
	virtual double get_transition_probability(const State * state_from, const Action * action, const State * state_to) const;
	virtual vector<Action const *> get_possible_actions(const State * state) const;

private:
	Graph graph;
	Graph::NodeMap<State *> state_map;
	Graph::ArcMap<Action const *> action_map;
	Graph::ArcMap<double> probability_map;
	Graph::ArcMap<bool> enable_map;

};

template <class State, class Action>
TransitionGraph<State,Action>::TransitionGraph():
				state_map(graph),
				action_map(graph),
				probability_map(graph),
				enable_map(graph)
{}

template <class State, class Action>
State TransitionGraph<State,Action>::add_state() {
	Node n = graph.addNode();
	State s(n);
	state_map[n] = s;
	return s;
}

template <class State, class Action>
void TransitionGraph<State,Action>::add_state(State * s) {
	Node n = graph.addNode();
	s->node_state_model.set_node(n);
	state_map[n] = s;
}

template <class State, class Action>
vector<State *> TransitionGraph<State,Action>::get_all_states() {
	vector<State *> states;
	for(Graph::NodeIt node(graph); node!=INVALID; ++node) {
		states.push_back(state_map[node]);
	}
	return states;
}

template <class State, class Action>
vector<State const *> TransitionGraph<State,Action>::get_adjacent_states(const State * s) {
    vector<State const *> states;
    State * other;
    uint state_idx;
    for(Graph::OutArcIt arc(graph,s->node_state_model.get_node()); arc!=INVALID; ++arc) {
        other = state_map[graph.oppositeNode(s->node_state_model.get_node(),arc)];
        for(state_idx = 0; state_idx<states.size(); ++state_idx) {
            if(other==states[state_idx]) break;
        }
        if(state_idx==states.size()) states.push_back(other);
    }
    return states;
}

template <class State, class Action>
vector<Action const *> TransitionGraph<State,Action>::get_available_actions(const State * s) {
	vector<Action const *> actions;
	uint action_idx;
	for(Graph::OutArcIt arc(graph,s->node_state_model.get_node()); arc!=INVALID; ++arc) {
		for(action_idx = 0; action_idx<actions.size(); ++action_idx) {
			if(action_map[arc]==actions[action_idx]) break;
		}
		if(action_idx==actions.size()) actions.push_back(action_map[arc]);
	}
	return actions;
}

template <class State, class Action>
void TransitionGraph<State,Action>::add_transition(const State * s1, const Action * a, const State * s2, const double& probability, const bool& enabled, const bool& normalize ) {
	Arc arc = graph.addArc(s1->node_state_model.get_node(), s2->node_state_model.get_node());
	action_map[arc] = a;
	probability_map[arc] = probability;
	enable_map[arc] = enabled;
	if(normalize) normalize_transitions(s1);
}
template <class State, class Action>
void TransitionGraph<State,Action>::remove_all_transitions() {
	for(Graph::ArcIt arc(graph); arc!=INVALID; ++arc) graph.erase(arc);
}

template <class State, class Action>
void TransitionGraph<State,Action>::normalize_transitions(const State * s) {
	vector<Action const *> actions;
	vector<double> probabilities;

	// sum probabilities for each action
	uint action_idx;
	for(Graph::OutArcIt arc(graph,s->node_state_model.get_node()); arc!=INVALID; ++arc) {
		// if transition is enabled
		if(enable_map[arc]) {
			// determine action index
			for(action_idx = 0; action_idx<actions.size(); ++action_idx) {
				if(actions[action_idx]==action_map[arc]) break;
			}
			// add new entry if action was not found
			if(action_idx==actions.size()) {
				actions.push_back(action_map[arc]);
				probabilities.push_back(0);
			}
			// sum probabilities
			probabilities[action_idx] += probability_map[arc];
		}
	}

	// normalize probabilities
	for(Graph::OutArcIt arc(graph,s->node_state_model.get_node()); arc!=INVALID; ++arc) {
		// if transition is enabled
		if(enable_map[arc]) {
			// determine action index
			for(action_idx = 0; action_idx<actions.size(); ++action_idx) {
				if(actions[action_idx]==action_map[arc]) break;
			}
			// normalize probabilities
			probability_map[arc] /= probabilities[action_idx];
		}
	}
}

template <class State, class Action>
void TransitionGraph<State,Action>::normalize_all_transitions() {
	for(Graph::NodeIt node(graph); node!=INVALID; ++node) normalize_transitions(state_map[node]);
}

template <class State, class Action>
void TransitionGraph<State,Action>::enable_transitions(const State * state_from, const State * state_to, const bool& normalize) {
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to ) enable_map[arc] = true;
	}
	if(normalize) normalize_transitions(state_from);
}

template <class State, class Action>
void TransitionGraph<State,Action>::enable_transitions(const State * state_from, const State * state_to, const Action * action, const bool& normalize) {
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to && action_map[arc]==action ) enable_map[arc] = true;
	}
	if(normalize) normalize_transitions(state_from);
}

template <class State, class Action>
void TransitionGraph<State,Action>::enable_bidirectional_transitions(const State * state_from, const State * state_to, const bool& normalize) {
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to ) enable_map[arc] = true;
	}
	for(Graph::InArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to ) enable_map[arc] = true;
	}
	if(normalize) normalize_transitions(state_from);
	if(normalize) normalize_transitions(state_to);
}

template <class State, class Action>
void TransitionGraph<State,Action>::enable_bidirectional_transitions(const State * state_from, const State * state_to, const Action * action, const bool& normalize) {
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to && action_map[arc]==action ) enable_map[arc] = true;
	}
	for(Graph::InArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to && action_map[arc]==action ) enable_map[arc] = true;
	}
	if(normalize) normalize_transitions(state_from);
	if(normalize) normalize_transitions(state_to);
}

template <class State, class Action>
void TransitionGraph<State,Action>::disable_transitions(const State * state_from, const State * state_to, const bool& normalize) {
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to ) enable_map[arc] = false;
	}
	if(normalize) normalize_transitions(state_from);
}

template <class State, class Action>
void TransitionGraph<State,Action>::disable_transitions(const State * state_from, const State * state_to, const Action * action, const bool& normalize) {
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to && action==action_map[arc] ) enable_map[arc] = false;
	}
	if(normalize) normalize_transitions(state_from);
}

template <class State, class Action>
void TransitionGraph<State,Action>::disable_bidirectional_transitions(const State * state_from, const State * state_to, const bool& normalize) {
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to ) enable_map[arc] = false;
	}
	for(Graph::InArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to ) enable_map[arc] = false;
	}
	if(normalize) normalize_transitions(state_from);
	if(normalize) normalize_transitions(state_to);
}

template <class State, class Action>
void TransitionGraph<State,Action>::disable_bidirectional_transitions(const State * state_from, const State * state_to, const Action * action, const bool& normalize) {
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to && action==action_map[arc] ) enable_map[arc] = false;
	}
	for(Graph::InArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)]==state_to && action==action_map[arc] ) enable_map[arc] = false;
	}
	if(normalize) normalize_transitions(state_from);
	if(normalize) normalize_transitions(state_to);
}

template <class State, class Action>
State * TransitionGraph<State,Action>::get_transition(State * const state_from, const Action * action) const {
	double r = drand48(), sum = 0;
	int count = 0, disabled_count = 0;
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		// if transition is enabled and action matches
		if(action_map[arc]==action) {
			if(enable_map[arc]) {
				sum += probability_map[arc];
				if( sum > r ) {
					DEBUG_OUT(2,"Performed transition");
					return state_map[graph.oppositeNode(state_from->node_state_model.get_node(),arc)];
				}
				++count;
			}
			else ++disabled_count;
		}
	}
	if(count==0) {
		if(disabled_count==0) { DEBUG_OUT(1,"No transition defined for this action."); }
		else { DEBUG_OUT(1, disabled_count << " transitions for this action defined but disabled."); }
	}
	else { DEBUG_OUT(1,"Transition(s) defined but no transition performed (probabilies not normalized)."); }
	return state_from;
}

template <class State, class Action>
double TransitionGraph<State,Action>::get_transition_probability(const State * state_from, const Action * action, const State * state_to) const {
	double prob = 0;
	for(Graph::OutArcIt arc(graph, state_from->node_state_model.get_node()); arc!=INVALID; ++arc) {
		if( state_map[graph.target(arc)]==state_to && enable_map[arc] && action_map[arc]==action) prob += probability_map[arc];
	}
	if( prob > 1) {
		DEBUG_OUT(0,"ERROR: Probability larger than one.");
		return 1;
	}
	return prob;
}

template <class State, class Action>
vector<Action const *> TransitionGraph<State,Action>::get_possible_actions(const State * state) const {
	vector<Action const *> actions;
	uint action_idx;
	for(Graph::OutArcIt arc(graph, state->node_state_model.get_node()); arc!=INVALID; ++arc) {
		// determine action index
		for(action_idx = 0; action_idx<actions.size(); ++action_idx) {
			if(actions[action_idx]==action_map[arc]) break;
		}
		// add new entry if action was not found
		if(action_idx==actions.size()) {
			actions.push_back(action_map[arc]);
		}
	}
	return actions;
}

#include "debug_exclude.h"

#endif /* TRANSITIONGRAPH_H_ */
