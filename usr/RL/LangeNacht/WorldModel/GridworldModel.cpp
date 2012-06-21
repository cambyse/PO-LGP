/*
 * GridworldModel.cpp
 *
 *  Created on: May 3, 2012
 *      Author: robert
 */

#include "GridworldModel.h"

#define DEBUG_STRING "GridworldModel: "
#define DEBUG_LEVEL 1
#include "debug.h"

#include <QtGui/QGraphicsScene>

#include <math.h>

using std::vector;

GridworldModel::Action GridworldModel::stay = Action('s');
GridworldModel::Action GridworldModel::right = Action('r');
GridworldModel::Action GridworldModel::left = Action('l');
GridworldModel::Action GridworldModel::down = Action('d');
GridworldModel::Action GridworldModel::up = Action('u');

GridworldModel::GridworldModel(const int& x, const int& y, const double& r, const double& d):
		x_size(x),
		y_size(y),
		agent_state(0),
		state_map(),
		valueFunction(&g,d),
		min_rew(0),
		max_rew(0),
		random(r)
{

	state_map.resize(x_size, vector<State>(y_size,State(0,0)));

	// add states
	for(int x=0; x<x_size; ++x)
		for(int y=0; y<y_size; ++y) {
			state_map[x][y] = State(x,y);
			g.add_state(&state_map[x][y]);
		}

	initialize_transitions();
	agent_state = &state_map[0][0];
	valueFunction.init();
}


void GridworldModel::display_all_states(QGraphicsScene * const scene, const char c, const bool& show_actions) {
    switch(c) {
    case 'r':
    {
        calculate_min_max_reward();
        double norm = fabs(max_rew)>fabs(min_rew) ? fabs(max_rew) : fabs(min_rew);
        DEBUG_OUT(2,"min/max reward: " << min_rew << "/" << max_rew );
        if(norm==0) norm = 1;
        double val;
        for(int x=0; x<x_size; ++x)
            for(int y=0; y<y_size; ++y) {
                val = state_map[x][y].state_action_value_state_model.state_reward/norm;
                DEBUG_OUT(2,"State reward: " << state_map[x][y].state_action_value_state_model.state_reward << ", min: " << min_rew << ", max: " << max_rew << ", color value: "<< val);
                if( val>0 ) state_map[x][y].grid_world_state_model.display(scene,QColor(245,245*(1-val),245*(1-val)),state_map[x][y].state_action_value_state_model.action_values,state_map[x][y].state_action_value_state_model.available_actions,show_actions);
                else state_map[x][y].grid_world_state_model.display(scene,QColor(245*(1+val),245*(1+val),245),state_map[x][y].state_action_value_state_model.action_values,state_map[x][y].state_action_value_state_model.available_actions,show_actions);
            }
    }
    break;
    case 'v':
    default:
    {
        double min = valueFunction.get_min_value();
        double max = valueFunction.get_max_value();
        double norm = fabs(max)>fabs(min) ? fabs(max) : fabs(min);
        DEBUG_OUT(2,"min/max value: " << min << "/" << max );
        if(norm==0) norm=1;
        double val;
        for(int x=0; x<x_size; ++x)
            for(int y=0; y<y_size; ++y) {
                val = state_map[x][y].state_action_value_state_model.state_value/norm;
                DEBUG_OUT(2,"State value: " << state_map[x][y].state_action_value_state_model.state_value << ", min: " << min << ", max: " << max << ", color value: "<< val);
                if( val>0 ) state_map[x][y].grid_world_state_model.display(scene,QColor(245,245*(1-val),245*(1-val)),state_map[x][y].state_action_value_state_model.action_values,state_map[x][y].state_action_value_state_model.available_actions,show_actions);
                else state_map[x][y].grid_world_state_model.display(scene,QColor(245*(1+val),245*(1+val),245),state_map[x][y].state_action_value_state_model.action_values,state_map[x][y].state_action_value_state_model.available_actions,show_actions);
            }
    }
    break;
    }
}

void GridworldModel::display_agent(QGraphicsScene * const scene) {
	double size = agent_state->grid_world_state_model.get_size();
	scene->addEllipse(
			agent_state->grid_world_state_model.get_x() - size/2 + 0.2*size,
			agent_state->grid_world_state_model.get_y() - size/2 + 0.2*size,
			0.6*size,
			0.6*size,
			QPen(QColor(0,0,0)),
			QBrush(QColor(0,255,0,180))
	);
}

void GridworldModel::set_current_state(int x, int y) {
	if(x < 0)       x = 0;
	if(x >= x_size) x = x_size-1;
	if(y < 0)       y = 0;
	if(y >= y_size) y = y_size-1;
	agent_state = &state_map[x][y];
}

void GridworldModel::get_current_state(int& x, int& y) {
	x = agent_state->grid_world_state_model.get_x();
	y = agent_state->grid_world_state_model.get_y();
}

void GridworldModel::add_wall(const int& x1, const int& y1, const int& x2, const int& y2) {

	// both state inside allowed bounds
	if( x1>=0 && x1<x_size && x2>=0 && x2<x_size && y1>=0 && y1<y_size && y2>=0 && y2<y_size ) {
		// s2 is left of s1
		if( x1-x2==+1 && y1-y2== 0 ) {
			state_map[x1][y1].grid_world_state_model.set_wall('l');
			state_map[x2][y2].grid_world_state_model.set_wall('r');
		}
		// s2 is right of s1
		else if( x1-x2==-1 && y1-y2== 0 ) {
			state_map[x1][y1].grid_world_state_model.set_wall('r');
			state_map[x2][y2].grid_world_state_model.set_wall('l');
		}
		// s2 is above of s1
		else if( x1-x2== 0 && y1-y2==+1 ) {
			state_map[x1][y1].grid_world_state_model.set_wall('t');
			state_map[x2][y2].grid_world_state_model.set_wall('b');
		}
		// s2 is below of s1
		else if( x1-x2== 0 && y1-y2==-1 ) {
			state_map[x1][y1].grid_world_state_model.set_wall('b');
			state_map[x2][y2].grid_world_state_model.set_wall('t');
		}
		else {
			DEBUG_OUT(0,"Error: Unable to add wall (states are not adjoined: (" << x1 << "," << y1 << ")/(" << x2 << "," << y2 << ") )");
			return;
		}
		std::pair<std::set<Wall>::iterator,bool> ret = walls.insert(Wall(x1,y1,x2,y2));
		if(ret.second) DEBUG_OUT(1,"Inserted wall (" << x1 << "," << y1 << "," << x2 << "," << y2 << ")");
		g.disable_bidirectional_transitions(&state_map[x1][y1], &state_map[x2][y2], false);
	} else {
		DEBUG_OUT(0,"Error: Unable to add wall (at least one state index is out of range: (" << x1 << "," << y1 << ")/(" << x2 << "," << y2 << ") )");
		return;
	}
}


void GridworldModel::remove_wall(const int& x1, const int& y1, const int& x2, const int& y2) {

	// both state inside allowed bounds
	if( x1>=0 && x1<x_size && x2>=0 && x2<x_size && y1>=0 && y1<y_size && y2>=0 && y2<y_size ) {
		// s2 is left of s1
		if( x1-x2==+1 && y1-y2== 0 ) {
			state_map[x1][y1].grid_world_state_model.unset_wall('l');
			state_map[x2][y2].grid_world_state_model.unset_wall('r');
		}
		// s2 is right of s1
		else if( x1-x2==-1 && y1-y2== 0 ) {
			state_map[x1][y1].grid_world_state_model.unset_wall('r');
			state_map[x2][y2].grid_world_state_model.unset_wall('l');
		}
		// s2 is above of s1
		else if( x1-x2== 0 && y1-y2==+1 ) {
			state_map[x1][y1].grid_world_state_model.unset_wall('t');
			state_map[x2][y2].grid_world_state_model.unset_wall('b');
		}
		// s2 is below of s1
		else if( x1-x2== 0 && y1-y2==-1 ) {
			state_map[x1][y1].grid_world_state_model.unset_wall('b');
			state_map[x2][y2].grid_world_state_model.unset_wall('t');
		}
		else {
			DEBUG_OUT(0,"Error: Unable to remove wall (states are not adjoined: (" << x1 << "," << y1 << ")/(" << x2 << "," << y2 << ") )");
			return;
		}
		int i = walls.erase(Wall(x1,y1,x2,y2));
		if(i>0) DEBUG_OUT(1,"Erased wall (" << x1 << "," << y1 << "," << x2 << "," << y2 << ")");
		g.enable_bidirectional_transitions(&state_map[x1][y1], &state_map[x2][y2], false);
	}
	else {
		DEBUG_OUT(0,"Error: Unable to remove wall (at least one state index is out of range: (" << x1 << "," << y1 << ")/(" << x2 << "," << y2 << ") )");
		return;
	}
}

void GridworldModel::remove_all_walls() {
	for(int x=0; x<x_size; ++x) {
			for(int y=0; y<y_size; ++y) {
				if(x+1<x_size) remove_wall(x,y,x+1,y);
				if(y+1<y_size) remove_wall(x,y,x,y+1);
			}
	}
}

void GridworldModel::set_reward(int x, int y, const double& r) {
	// state inside allowed bounds
	if( x>=0 && x<x_size && y>=0 && y<y_size ) {
		state_map[x][y].state_action_value_state_model.state_reward = r;
	}
	else {
		DEBUG_OUT(0,"Error: Unable to set reward (at least one state index is out of range: (" << x << "," << y << ") )" );
		return;
	}
}

double GridworldModel::get_reward(int x, int y) {
	// state inside allowed bounds
	if( x>=0 && x<x_size && y>=0 && y<y_size ) {
		return state_map[x][y].state_action_value_state_model.state_reward;
	}
	else {
		DEBUG_OUT(0,"Error: Unable to get reward (at least one state index is out of range: (" << x << "," << y << ") )" );
		return 0;
	}
}

void GridworldModel::delete_all_rewards() {
    for(int x=0; x<x_size; ++x)
        for(int y=0; y<y_size; ++y)
            state_map[x][y].state_action_value_state_model.state_reward = 0;
}

void GridworldModel::set_random(const double& r) {
	random = r;
	initialize_transitions();
//	valueFunction.init();
}

void GridworldModel::perform_transition(const char c) {
    switch(c) {
    case 'S':
    case 's':
        agent_state = g.get_transition(agent_state, &stay);
        break;
    case 'L':
    case 'l':
        agent_state = g.get_transition(agent_state, &left);
        break;
    case 'R':
    case 'r':
        agent_state = g.get_transition(agent_state, &right);
        break;
    case 'U':
    case 'u':
        agent_state = g.get_transition(agent_state, &up);
        break;
    case 'D':
    case 'd':
        agent_state = g.get_transition(agent_state, &down);
        break;
    default:
        DEBUG_OUT(0,"No transition performed: unknown action.");
        break;
    }
}

void GridworldModel::perform_optimal_transition() {
	Action const * a;
	double max_val = -DBL_MAX;
	vector<Action const *> av_a = agent_state->state_action_value_state_model.available_actions;
	vector<double> a_vals = agent_state->state_action_value_state_model.action_values;
	for(uint idx=0; idx<av_a.size(); ++idx) {
		if(a_vals[idx]>max_val) {
			max_val = a_vals[idx];
			a = av_a[idx];
		}
	}
	agent_state = g.get_transition(agent_state, a);
}

void GridworldModel::initialize_transitions() {
	g.remove_all_transitions();
	// add transitions
	for(int x=0; x<x_size; ++x) {
		for(int y=0; y<y_size; ++y) {

			// add planned actions
			g.add_transition(&state_map[x][y], &stay, &state_map[x  ][y  ], 1-random, true, false); // stay
			if(x+1<x_size) g.add_transition(&state_map[x][y], &right, &state_map[x+1][y  ], 1-random, true, false); // right
			if(x-1>=0)     g.add_transition(&state_map[x][y], &left, &state_map[x-1][y  ], 1-random, true, false); // left
			if(y+1<y_size) g.add_transition(&state_map[x][y], &down, &state_map[x  ][y+1], 1-random, true, false); // down
			if(y-1>=0)     g.add_transition(&state_map[x][y], &up, &state_map[x  ][y-1], 1-random, true, false); // up

			// include random actions
			//			if(random!=0.0) { // remove this so value functions do not need to be reinitialized
			double p = random/GridworldActionModel::get_number_of_actions();
			g.add_transition(&state_map[x][y], &stay, &state_map[x  ][y  ], p, true, false); // stay
			g.add_transition(&state_map[x][y], &right, &state_map[x  ][y  ], p, true, false); // stay
			g.add_transition(&state_map[x][y], &left, &state_map[x  ][y  ], p, true, false); // stay
			g.add_transition(&state_map[x][y], &up, &state_map[x  ][y  ], p, true, false); // stay
			g.add_transition(&state_map[x][y], &down, &state_map[x  ][y  ], p, true, false); // stay
			if(x+1<x_size) {
				g.add_transition(&state_map[x][y], &stay, &state_map[x+1][y  ], p, true, false); // right
				g.add_transition(&state_map[x][y], &right, &state_map[x+1][y  ], p, true, false); // right
				g.add_transition(&state_map[x][y], &left, &state_map[x+1][y  ], p, true, false); // right
				g.add_transition(&state_map[x][y], &up, &state_map[x+1][y  ], p, true, false); // right
				g.add_transition(&state_map[x][y], &down, &state_map[x+1][y  ], p, true, false); // right
			}
			if(x-1>=0) {
				g.add_transition(&state_map[x][y], &stay, &state_map[x-1][y  ], p, true, false); // left
				g.add_transition(&state_map[x][y], &right, &state_map[x-1][y  ], p, true, false); // left
				g.add_transition(&state_map[x][y], &left, &state_map[x-1][y  ], p, true, false); // left
				g.add_transition(&state_map[x][y], &up, &state_map[x-1][y  ], p, true, false); // left
				g.add_transition(&state_map[x][y], &down, &state_map[x-1][y  ], p, true, false); // left
			}
			if(y+1<y_size) {
				g.add_transition(&state_map[x][y], &stay, &state_map[x  ][y+1], p, true, false); // down
				g.add_transition(&state_map[x][y], &right, &state_map[x  ][y+1], p, true, false); // down
				g.add_transition(&state_map[x][y], &left, &state_map[x  ][y+1], p, true, false); // down
				g.add_transition(&state_map[x][y], &up, &state_map[x  ][y+1], p, true, false); // down
				g.add_transition(&state_map[x][y], &down, &state_map[x  ][y+1], p, true, false); // down
			}
			if(y-1>=0) {
				g.add_transition(&state_map[x][y], &stay, &state_map[x  ][y-1], p, true, false); // up
				g.add_transition(&state_map[x][y], &right, &state_map[x  ][y-1], p, true, false); // up
				g.add_transition(&state_map[x][y], &left, &state_map[x  ][y-1], p, true, false); // up
				g.add_transition(&state_map[x][y], &up, &state_map[x  ][y-1], p, true, false); // up
				g.add_transition(&state_map[x][y], &down, &state_map[x  ][y-1], p, true, false); // up
			}
			//			}
		}
	}
	g.normalize_all_transitions();
	rebuild_walls();
}

void GridworldModel::rebuild_walls() {
	std::set<Wall>::iterator it;
	for(it=walls.begin(); it!=walls.end(); it++) {
		add_wall((*it).x1,(*it).y1,(*it).x2,(*it).y2);
	}
}

void GridworldModel::calculate_min_max_reward() {
    min_rew =  DBL_MAX;
    max_rew = -DBL_MAX;
    double rew;
    for(int x=0; x<x_size; ++x)
        for(int y=0; y<y_size; ++y) {
            rew = state_map[x][y].state_action_value_state_model.state_reward;
            min_rew = rew<min_rew ? rew : min_rew;
            max_rew = rew>max_rew ? rew : max_rew;
        }
}
