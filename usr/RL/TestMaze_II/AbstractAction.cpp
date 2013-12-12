#include "AbstractAction.h"

#include "debug.h"

AbstractAction::AbstractAction(ACTION_TYPE t):
    action_type(t)
{}

AbstractAction::Iterator AbstractAction::begin() const {
    return Iterator(ptr_t(new AbstractAction()));
}

AbstractAction::ptr_t AbstractAction::next() const {
    return ptr_t(new AbstractAction());
}

bool AbstractAction::operator!=(const AbstractIteratableSpace& other) const {
    auto abstract_action = dynamic_cast<const AbstractAction *>(&other);
    if(abstract_action==nullptr) {
        DEBUG_ERROR("Dynamic cast failed");
        return true;
    } else {
        return *this!=*abstract_action;
    }
}

bool AbstractAction::operator!=(const AbstractAction& other) const {
    return this->action_type!=other.action_type;
}

bool AbstractAction::operator<(const AbstractIteratableSpace& other) const {
    auto abstract_action = dynamic_cast<const AbstractAction *>(&other);
    if(abstract_action==nullptr) {
        DEBUG_ERROR("Dynamic cast failed");
        return true;
    } else {
        return *this<*abstract_action;
    }
}

bool AbstractAction::operator<(const AbstractAction& other) const {
    return this->action_type<other.action_type;
}

const char * AbstractAction::print() const {
    return "AbstractAction()";
}

AbstractAction::ACTION_TYPE AbstractAction::get_type() const {
    return action_type;
}

void AbstractAction::set_type(ACTION_TYPE t) {
    action_type = t;
}
