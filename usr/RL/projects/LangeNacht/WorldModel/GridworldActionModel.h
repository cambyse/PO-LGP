/*
 * GridworldActionModel.h
 *
 *  Created on: May 3, 2012
 *      Author: robert
 */

#ifndef GRIDWORLDACTIONMODEL_H_
#define GRIDWORLDACTIONMODEL_H_

class GridworldActionModel {

public:

	enum ACTION_TYPE { STAY, UP, DOWN, LEFT, RIGHT };
	static int get_number_of_actions() { return 5; }

	GridworldActionModel(const char a = 's') {
		switch(a)
		{
		case 'U':
		case 'u':
			action = UP;
			break;
		case 'D':
		case 'd':
			action = DOWN;
			break;
		case 'R':
		case 'r':
			action = LEFT;
			break;
		case 'L':
		case 'l':
			action = RIGHT;
			break;
		case 'S':
		case 's':
		default:
			action = STAY;
			break;
		}
	}

	virtual ~GridworldActionModel() {}

	virtual bool operator==(const GridworldActionModel& other) const { return action==other.get_action(); }
	virtual bool operator!=(const GridworldActionModel& other) const { return !(*this==other); }

	ACTION_TYPE get_action() const { return action; }

	void set_action(const ACTION_TYPE& a) { action = a; }
	void set_action(const char a = 's') {
		switch(a)
		{
		case 'U':
		case 'u':
			action = UP;
			break;
		case 'D':
		case 'd':
			action = DOWN;
			break;
		case 'R':
		case 'r':
			action = LEFT;
			break;
		case 'L':
		case 'l':
			action = RIGHT;
			break;
		case 'S':
		case 's':
		default:
			action = STAY;
			break;
		}
	}

private:

	ACTION_TYPE action;
};

#endif /* GRIDWORLDACTIONMODEL_H_ */
