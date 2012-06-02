/*
 * GridworldModel.h
 *
 *  Created on: May 3, 2012
 *      Author: robert
 */

#ifndef GRIDWORLDMODEL_H_
#define GRIDWORLDMODEL_H_

#include "TransitionGraph.h"
#include "GridworldStateModel.h"
#include "NodeStateModel.h"
#include "AbstractState.h"
#include "GridworldActionModel.h"
#include "StateActionValueFunction.h"
#include "StateActionValueStateModel.h"

#include <vector>
#include <set>

class QGraphicsScene;

class GridworldModel {

public:

	typedef GridworldActionModel Action;

	class State: public AbstractState {
	public:
		State(const int& x = 0, const int& y = 0, const Node& n = INVALID):
			grid_world_state_model(x,y,0.9), node_state_model(n) {};
		~State() {};
		static bool has_grid_world_state_model() { return true; }
		static bool has_node_state_model() { return true; }
		static bool has_state_action_value_state_model() { return true; }
		GridworldStateModel grid_world_state_model;
		NodeStateModel node_state_model;
		StateActionValueStateModel<State, Action> state_action_value_state_model;
	};


	GridworldModel(const int& x, const int& y, const double& r = 0.0, const double& d = 0.9);

	virtual ~GridworldModel() {}

	void display_all_states(QGraphicsScene * const scene, const char c, const bool& show_actions);

	void display_agent(QGraphicsScene * const scene);

	void set_current_state(int x, int y);

	void add_wall(const int& x1, const int& y1, const int& x2, const int& y2);
	void remove_wall(const int& x1, const int& y1, const int& x2, const int& y2);
	void remove_all_walls();


	void set_reward(int x, int y, const double& r);
	double get_reward(int x, int y);
	void delete_all_rewards();

	void set_random(const double& r);

	void perform_transition(const char c);
	void perform_optimal_transition();

	void iterate_value_function() { valueFunction.iterate(); }

	void set_discount(const double& d) { valueFunction.set_discount(d); }

private:

	class Wall {
	public:
		Wall(const int& xx1, const int& yy1, const int& xx2, const int& yy2) {
			if( xx1<xx2 || (xx1==xx2 && yy1<yy2) ) {
				x1=xx1;
				y1=yy1;
				x2=xx2;
				y2=yy2;
			} else {
				x1=xx2;
				y1=yy2;
				x2=xx1;
				y2=yy1;
			}
		}
		bool operator<(const Wall& other) const {
			if(x1<other.x1) return true;
			else if(x1>other.x1) return false;
			else if(y1<other.y1) return true;
			else if(y1>other.y1) return false;
			else if(x2<other.x2) return true;
			else if(x2>other.x2) return false;
			else if(y2<other.y2) return true;
			else if(y2>other.y2) return false;
			else return false;
		}
		bool operator==(const Wall& other) const {
			return ( x1==other.x1 && y1==other.y1 && x2==other.x2 && y2==other.y2 ) ||
					( x1==other.x2 && y1==other.y2 && x2==other.x1 && y2==other.y1 );
		}
		int x1,y1,x2,y2;
	};

	void initialize_transitions();

	void rebuild_walls();

	void calculate_min_max_reward();

	TransitionGraph<State,Action> g;
	int x_size, y_size;
	State * agent_state;
	std::vector<std::vector<State> > state_map;
	StateActionValueFunction<State,Action> valueFunction;
	Action static stay, right, left, down, up;
	double min_rew, max_rew, random;
	std::set<Wall> walls;

};

#endif /* GRIDWORLDMODEL_H_ */
