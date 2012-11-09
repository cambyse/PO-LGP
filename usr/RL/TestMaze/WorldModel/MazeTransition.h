/*
 * MazeTransition.h
 *
 *  Created on: Jun 21, 2012
 *      Author: robert
 */

#ifndef MAZETRANSITION_H_
#define MAZETRANSITION_H_

#include "MazeState.h"
#include "MazeAction.h"

#include <vector>

namespace WorldModel
{

template <class State = MazeState, class Action = MazeAction>
class MazeTransition
{

public:

    MazeTransition() {}
    virtual ~MazeTransition() {}

    double transition_probability(const State& initial_state, const Action& action, const State& final_state) const {
        switch(action.get_action()) {
        case Action::LEFT:
            if( final_state.is_left_of(initial_state) ) return 1;
            else return 0;
            break;
        case Action::RIGHT:
            if( final_state.is_right_of(initial_state) ) return 1;
            else return 0;
            break;
        case Action::UP:
            if( final_state.is_above(initial_state) ) return 1;
            else return 0;
            break;
        case Action::DOWN:
            if( final_state.is_below(initial_state) ) return 1;
            else return 0;
            break;
        case Action::STAY:
            if( final_state.equal_x_y(initial_state) ) return 1;
            else return 0;
            break;
        default:
            return 0;
        }
    }

    std::vector<State>  get_target_states(const State& initial_state, const int& x_max = -1, const int& y_max = -1) const {
        std::vector<State> ret;
        int x, y;
        initial_state.get_x_y_pos(x,y);
        if(x_max==-1 || x<x_max) ret.push_back(State(x+1,y  ));
        if(y_max==-1 || y<y_max) ret.push_back(State(x  ,y+1));
        if(x>0                 ) ret.push_back(State(x-1,y  ));
        if(y>0                 ) ret.push_back(State(x  ,y-1));
        return ret;
    }
    std::vector<State>  get_source_states(const State& final_state, const int& x_max = -1, const int& y_max = -1) const {
        std::vector<State> ret;
        int x, y;
        final_state.get_x_y_pos(x,y);
        if(x_max==-1 || x<x_max) ret.push_back(State(x+1,y  ));
        if(y_max==-1 || y<y_max) ret.push_back(State(x  ,y+1));
        if(x>0                 ) ret.push_back(State(x-1,y  ));
        if(y>0                 ) ret.push_back(State(x  ,y-1));
        return ret;
    }
    std::vector<Action> get_actions_from_state(const State& initial_state, const int& x_max = -1, const int& y_max = -1) const {
        std::vector<Action> ret;
        int x, y;
        initial_state.get_x_y_pos(x,y);
        if(x_max==-1 || x<x_max) ret.push_back(Action('r'));
        if(y_max==-1 || y<y_max) ret.push_back(Action('d'));
        if(x>0                 ) ret.push_back(Action('l'));
        if(y>0                 ) ret.push_back(Action('u'));
        return ret;
    }
    std::vector<Action> get_actions_to_state(const State& final_state, const int& x_max = -1, const int& y_max = -1) const {
        std::vector<Action> ret;
        int x, y;
        final_state.get_x_y_pos(x,y);
        if(x_max==-1 || x<x_max) ret.push_back(Action('l'));
        if(y_max==-1 || y<y_max) ret.push_back(Action('u'));
        if(x>0                 ) ret.push_back(Action('r'));
        if(y>0                 ) ret.push_back(Action('d'));
        return ret;
    }

private:

};

} /* namespace WorldModel */
#endif /* MAZETRANSITION_H_ */
