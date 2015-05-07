/*
 * AbstractState.h
 *
 *  Created on: May 9, 2012
 *      Author: robert
 */

#ifndef ABSTRACTSTATE_H_
#define ABSTRACTSTATE_H_

class AbstractState {

public:

	AbstractState() {}
	virtual ~AbstractState() {}

	static bool has_grid_world_state_model() { return false; }
	static bool has_node_state_model() { return false; }
	static bool has_state_action_value_state_model() { return false; }
};

#endif /* ABSTRACTSTATE_H_ */
