#include "AugmentedMazeAction.h"

#include "../util/Macro.h"

#include "../debug.h"

using std::string;

AugmentedMazeAction::AugmentedMazeAction():
    MazeAction(),
    tag(TAG::TAG_0)
{
    set_type(ACTION_TYPE::AUGMENTED_MAZE_ACTION);
}

AugmentedMazeAction::AugmentedMazeAction(ACTION a, TAG t):
    MazeAction(a),
    tag(t)
{
    set_type(ACTION_TYPE::AUGMENTED_MAZE_ACTION);
}

AugmentedMazeAction::ptr_t AugmentedMazeAction::next() const {
    // use (temporarily) incremented action and tag
    ACTION a = (ACTION)((int)action+1);
    TAG t = (TAG)((int)tag+1);
    if(a>=ACTION::END) {   // action over limit
        if(t<TAG::END) {  // tag not over limit
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
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const AugmentedMazeAction *);
    return ( this->action!=ptr->action || this->tag!=ptr->tag );
}

bool AugmentedMazeAction::operator<(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const AugmentedMazeAction *);
    return ( this->action<ptr->action ||
             (this->action==ptr->action && this->tag<ptr->tag)
        );
}

const string AugmentedMazeAction::print() const {
    string ret("AugmentedMazeAction(");
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
        DEBUG_ERROR("Invalid tag");
        ret+="INVALID";
        break;
    }
    ret+=")";
    return ret;
}
