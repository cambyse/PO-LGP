#include "MazeAction.h"

#include "../debug.h"

using std::string;

MazeAction::MazeAction(ACTION a):
    action(a)
{
    set_type(ACTION_TYPE::MAZE_ACTION);
}

MazeAction::MazeAction(const char * a) {
    // set type
    set_type(ACTION_TYPE::MAZE_ACTION);
    // set action
    if(!strcmp(a,"up") || !strcmp(a,"u")) {
        action = ACTION::UP;
    } else if(!strcmp(a,"down") || !strcmp(a,"d")) {
        action = ACTION::DOWN;
    } else if(!strcmp(a,"left") || !strcmp(a,"l")) {
        action = ACTION::LEFT;
    } else if(!strcmp(a,"right") || !strcmp(a,"r")) {
        action = ACTION::RIGHT;
    } else if(!strcmp(a,"stay") || !strcmp(a,"s")) {
        action = ACTION::STAY;
    } else {
        DEBUG_ERROR("Not valid action ('" << a << "'");
        action = ACTION::END;
    }
}

MazeAction::ptr_t MazeAction::next() const {
    ACTION a = (ACTION)((int)action+1);
    if(a>=ACTION::END) {
        return ptr_t(new AbstractAction());
    } else {
        return ptr_t(new MazeAction(a));
    }
}

bool MazeAction::operator!=(const AbstractAction &other) const {
    if(this->get_type()!=other.get_type()) {
        return true;
    } else {
        auto maze_action = dynamic_cast<const MazeAction *>(&other);
        if(maze_action==nullptr) {
            DEBUG_ERROR("Dynamic cast failed");
            return true;
        } else {
            return this->action!=maze_action->action;
        }
    }
}

bool MazeAction::operator<(const AbstractAction &other) const {
    if(this->get_type()<other.get_type()) {
        return true;
    } else {
        auto maze_action = dynamic_cast<const MazeAction *>(&other);
        if(maze_action==nullptr) {
            DEBUG_ERROR("Dynamic cast failed");
            return true;
        } else {
            return this->action<maze_action->action;
        }
    }
}

const string MazeAction::print() const {
    string ret("MazeAction(");
    switch(action) {
    case ACTION::UP:
        ret+="   UP";
        break;
    case ACTION::DOWN:
        ret+=" DOWN";
        break;
    case ACTION::LEFT:
        ret+=" LEFT";
        break;
    case ACTION::RIGHT:
        ret+="RIGHT";
        break;
    case ACTION::STAY:
        ret+=" STAY";
        break;
    default:
        DEBUG_ERROR("Invalid action");
        ret+="INVALID";
        break;
    }
    ret+=")";
    return ret;
}

void MazeAction::set_type(ACTION_TYPE t) {
    AbstractAction::set_type(t);
}
