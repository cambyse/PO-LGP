#include "MazeAction.h"

#include "../debug.h"

MazeAction::MazeAction(ACTION a):
    action(a)
{
    set_type(ACTION_TYPE::MAZE_ACTION);
}

MazeAction::MazeAction(const char * a) {
    if(!strcmp(a,"up") || !strcmp(a,"u")) {
        action = ACTION::UP;
    } else if(!strcmp(a,"down") || !strcmp(a,"d")) {
        action = ACTION::UP;
    } else if(!strcmp(a,"left") || !strcmp(a,"l")) {
        action = ACTION::UP;
    } else if(!strcmp(a,"right") || !strcmp(a,"r")) {
        action = ACTION::UP;
    } else if(!strcmp(a,"stay") || !strcmp(a,"s")) {
        action = ACTION::UP;
    } else {
        DEBUG_ERROR("Not valid action ('" << a << "'");
        action = ACTION::NONE;
    }
}

MazeAction::Iterator MazeAction::begin() const {
    return Iterator(ptr_t(new MazeAction(ACTION::UP)));
}

MazeAction::ptr_t MazeAction::next() const {
    return ptr_t(new MazeAction((ACTION)((int)action+1)));
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

const char * MazeAction::print() const {
    std::string ret("MazeAction(");
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
        ret+=" NONE";
        break;
    }
    ret+=")";
    return ret.c_str();
}

void MazeAction::set_type(ACTION_TYPE t) {
    AbstractAction::set_type(t);
}
