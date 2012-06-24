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
#include "../Utils/Identifier.h"

namespace WorldModel
{

template <class State = MazeState<>, class Action = MazeAction>
class MazeTransition
{

public:

    MazeTransition() {}
    virtual ~MazeTransition() {}

private:

    /* Use 2D grid position as identifyer */
    class Location2D: public Utils::Identifier2D<int> {
        Location2D(const Utils::Identifier2D<int> i): Utils::Identifier2D<int>(i) {}
        int get_x_pos() const { return type1; }
        int get_y_pos() const { return type2; }
        Location2D shift_left()  { --(this->type1); return *this; }
        Location2D shift_right() { ++(this->type1); return *this; }
        Location2D shift_up()    { --(this->type2); return *this; }
        Location2D shift_down()  { ++(this->type2); return *this; }
    };

public:

    State perform_transition(const State& initial_state, const Action& action) {
        if(     action == Action('l')) return State( Location2D(initial_state.get_id()).shift_left(),  initial_state.get_walls());
        else if(action == Action('r')) return State( Location2D(initial_state.get_id()).shift_right(), initial_state.get_walls());
        else if(action == Action('u')) return State( Location2D(initial_state.get_id()).shift_up(),    initial_state.get_walls());
        else if(action == Action('d')) return State( Location2D(initial_state.get_id()).shift_down(),  initial_state.get_walls());
        else return initial_state;
    }

private:

};

} /* namespace WorldModel */
#endif /* MAZETRANSITION_H_ */
