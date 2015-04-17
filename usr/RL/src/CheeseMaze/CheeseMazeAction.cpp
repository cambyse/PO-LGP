#include "CheeseMazeAction.h"

#include "../util/debug.h"

using std::string;

#define USE_UTF8

CheeseMazeAction::CheeseMazeAction(ACTION a):
    action(a)
{
    set_type(ACTION_TYPE::CHEESE_MAZE_ACTION);
}

CheeseMazeAction::CheeseMazeAction(const string a) {
    // set type
    set_type(ACTION_TYPE::CHEESE_MAZE_ACTION);
    // set action
    if(a=="north" || a=="n") {
        action = ACTION::NORTH;
    } else if(a=="south" || a=="s") {
        action = ACTION::SOUTH;
    } else if(a=="west" || a=="w") {
        action = ACTION::WEST;
    } else if(a=="east" || a=="e") {
        action = ACTION::EAST;
    } else {
        DEBUG_ERROR("Not valid action: '" << a << "'");
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
    string ret;
    if(!print_short_name) {
        ret += "CheeseMazeAction(";
    }
    switch(action) {
#ifdef USE_UTF8
    case ACTION::NORTH:
        ret+="↑";
        break;
    case ACTION::SOUTH:
        ret+="↓";
        break;
    case ACTION::WEST:
        ret+="←";
        break;
    case ACTION::EAST:
        ret+="→";
        break;
#else
    case ACTION::NORTH:
        ret+="NORTH";
        break;
    case ACTION::SOUTH:
        ret+="SOUTH";
        break;
    case ACTION::WEST:
        ret+=" WEST";
        break;
    case ACTION::EAST:
        ret+=" EAST";
        break;
#endif
    default:
        DEBUG_ERROR("Invalid action");
        ret+="INVALID";
        break;
    }
    if(!print_short_name) {
        ret+=")";
    }
    return ret;
}

void CheeseMazeAction::set_type(ACTION_TYPE t) {
    AbstractAction::set_type(t);
}
