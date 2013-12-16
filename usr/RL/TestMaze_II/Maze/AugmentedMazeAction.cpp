#include "AugmentedMazeAction.h"

#include "../debug.h"

AugmentedMazeAction::AugmentedMazeAction(ACTION a, TAG t):
    MazeAction(a),
    tag(t)
{
    set_type(ACTION_TYPE::AUGMENTED_MAZE_ACTION);
}

AugmentedMazeAction::Iterator AugmentedMazeAction::begin() const {
    return Iterator(ptr_t(new AugmentedMazeAction(ACTION::UP,TAG::TAG_0)));
}

AugmentedMazeAction::ptr_t AugmentedMazeAction::next() const {
    // use (temporarily) incremented action and tag
    ACTION a = (ACTION)((int)action+1);
    TAG t = (TAG)((int)tag+1);
    if(a==ACTION::NONE) {   // action over limit
        if(t!=TAG::NONE) {  // tag not over limit
            a = ACTION::UP; // reset action to first
        } else {            // both over limit
            return ptr_t(new AbstractAction());
        }
    } else {                // action not over limit
        t = tag;            // reverse tag increment
    }
    return ptr_t(new AugmentedMazeAction(a,t));
}

bool AugmentedMazeAction::operator!=(const AbstractAction &other) const {
    if(this->get_type()!=other.get_type()) {
        return true;
    } else {
        auto augmented_maze_action = dynamic_cast<const AugmentedMazeAction *>(&other);
        if(augmented_maze_action==nullptr) {
            DEBUG_ERROR("Dynamic cast failed");
            return true;
        } else {
            return ( this->action!=augmented_maze_action->action ||
                     this->tag!=augmented_maze_action->tag );
        }
    }
}

bool AugmentedMazeAction::operator<(const AbstractAction &other) const {
    if(this->get_type()<other.get_type()) {
        return true;
    } else {
        auto augmented_maze_action = dynamic_cast<const AugmentedMazeAction *>(&other);
        if(augmented_maze_action==nullptr) {
            DEBUG_ERROR("Dynamic cast failed");
            return true;
        } else {
            return ( this->action<augmented_maze_action->action || (
                         this->action==augmented_maze_action->action &&
                         this->tag<augmented_maze_action->tag
                         ));
        }
    }
}

const char * AugmentedMazeAction::print() const {
    std::string ret("AugmentedMazeAction(");
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
    ret+=",";
    switch(tag) {
    case TAG::TAG_0:
        ret+="TAG_0";
        break;
    case TAG::TAG_1:
        ret+="TAG_1";
        break;
    case TAG::TAG_2:
        ret+="TAG_2";
        break;
    default:
        ret+=" NONE";
        break;
    }
    ret+=")";
    return ret.c_str();
}
