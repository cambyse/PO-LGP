#include "CheeseMazeAction.h"

#include "../util/Macro.h"

#include "../debug.h"

using std::string;

CheeseMazeAction::CheeseMazeAction(ACTION a):
    action(a)
{
    set_type(ACTION_TYPE::CHEESE_MAZE_ACTION);
}

CheeseMazeAction::CheeseMazeAction(const char * a) {
    // set type
    set_type(ACTION_TYPE::CHEESE_MAZE_ACTION);
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

CheeseMazeAction::ptr_t CheeseMazeAction::next() const {
    ACTION a = (ACTION)((int)action+1);
    if(a>=ACTION::END) {
        return ptr_t(new AbstractAction());
    } else {
        return ptr_t(new CheeseMazeAction(a));
    }
}

bool CheeseMazeAction::operator!=(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const CheeseMazeAction *);
    return this->action!=ptr->action;
}

bool CheeseMazeAction::operator<(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const CheeseMazeAction *);
    return this->action < ptr->action;
}

const string CheeseMazeAction::print() const {
    string ret("CheeseMazeAction(");
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

void CheeseMazeAction::set_type(ACTION_TYPE t) {
    AbstractAction::set_type(t);
}
