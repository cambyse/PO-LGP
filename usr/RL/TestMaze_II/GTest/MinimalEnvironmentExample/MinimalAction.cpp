#include "MinimalAction.h"

#include "../../util/Macro.h"

#include "../../debug.h"

MinimalAction::MinimalAction(ACTION a) {
    action = a;
    set_type(ACTION_TYPE::MINIMAL);
}

MinimalAction::ptr_t MinimalAction::next() const {
    if(action==CHANGE) {
        return ptr_t(new MinimalAction(STAY));
    } else {
        return ptr_t(new AbstractAction());
    }
}

bool MinimalAction::operator!=(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const MinimalAction *);
    return this->action!=ptr->action;
}

bool MinimalAction::operator<(const AbstractAction &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const MinimalAction *);
    return this->action < ptr->action;
}

const std::string MinimalAction::print() const {
    switch(action) {
    case ACTION::CHANGE:
        return std::string("MinimalAction(CHANGE)");
    case ACTION::STAY:
        return std::string("MinimalAction(STAY)");
    default:
        DEBUG_DEAD_LINE;
        return std::string("MinimalAction(INVALID)");
    }
}
