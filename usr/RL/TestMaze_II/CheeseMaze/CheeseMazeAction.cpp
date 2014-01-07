#include "CheseMazeAction.h"

#include "../util/Macro.h"

#include "../debug.h"

using std::string;

CheseMazeAction::CheseMazeAction(ACTION a):
    action(a)
{
    set_type(ACTION_TYPE::CHESE_MAZE_ACTION);
}

CheseMazeAction::CheseMazeAction(const char * a) {
    // set type
    set_type(ACTION_TYPE::CHESEMAZE_ACTION);
    // set action
    if(!strcmp(a,"north") || !strcmp(a,"n")) {
        action = ACTION::NORTH;
    } else if(!strcmp(a,"south") || !strcmp(a,"s")) {
        action = ACTION::SOUTH;
    } else if(!strcmp(a,"west") || !strcmp(a,"w")) {
        action = ACTION::WEST;
    } else if(!strcmp(a,"east") || !strcmp(a,"e")) {
        action = ACTION::EAST;
    } else {
        DEBUG_ERROR("Not valid action ('" << a << "'");
        action = ACTION::END;
    }
}

CheseMazeAction::ptr_t CheseMazeAction::next() const {
    ACTION a = (ACTION)((int)action+1);
    if(a>=ACTION::END) {
        return ptr_t(new AbstractAction());
    } else {
        return ptr_t(new CheseMazeAction(a));
    }
}

bool CheseMazeAction::operator!=(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const CheseMazeAction *);
    return this->action!=ptr->action;
}

bool CheseMazeAction::operator<(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const CheseMazeAction *);
    return this->action < ptr->action;
}

const string CheseMazeAction::print() const {
    string ret("CheseMazeAction(");
    switch(action) {
    case ACTION::NORTH:
        ret+="   NORTH";
        break;
    case ACTION::SOUTH:
        ret+=" SOUTH";
        break;
    case ACTION::WEST:
        ret+=" WEST";
        break;
    case ACTION::EAST:
        ret+="EAST";
        break;
    default:
        DEBUG_ERROR("Invalid action");
        ret+="INVALID";
        break;
    }
    ret+=")";
    return ret;
}

void CheseMazeAction::set_type(ACTION_TYPE t) {
    AbstractAction::set_type(t);
}
