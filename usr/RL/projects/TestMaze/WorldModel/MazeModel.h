/*
 * MazeModel.h
 *
 *  Created on: Jun 21, 2012
 *      Author: robert
 */

#ifndef MAZEMODEL_H_
#define MAZEMODEL_H_

#include "../WorldModel/MazeState.h"
#include "../WorldModel/MazeAction.h"
#include "../WorldModel/MazeTransition.h"

namespace WorldModel
{

template<class State = MazeState, class Action = MazeAction, class Transition = MazeTransition<> >
class MazeModel
{

public:
    typedef State StateType;
    typedef Action ActionType;
    typedef Transition TransitionType;

    MazeModel(const int& xSize = 0, const int& ySize = 0, const int& xPos = 0, const int& yPos = 0):
        x_size(xSize), y_size(ySize), agent_x_pos(xPos), agent_y_pos(yPos) {}
    virtual ~MazeModel() {}

    // Moving.
    void go_left()  { if(agent_x_pos>0       ) --agent_x_pos; }
    void go_right() { if(agent_x_pos<x_size-1) ++agent_x_pos; }
    void go_up()    { if(agent_y_pos>0       ) --agent_y_pos; }
    void go_down()  { if(agent_y_pos<y_size-1) ++agent_y_pos; }

    // Agent position.
    void get_x_y_pos(int& x, int& y) const { x=agent_x_pos; y=agent_y_pos; }
    int get_x_pos() const { return agent_x_pos; }
    int get_y_pos() const { return agent_y_pos; }

private:
    int x_size, y_size, agent_x_pos, agent_y_pos;
};

} /* namespace WorldModel */
#endif /* MAZEMODEL_H_ */
