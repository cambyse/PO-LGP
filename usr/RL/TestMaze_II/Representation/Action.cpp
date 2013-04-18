#include "Action.h"

using util::INVALID;

Action::Action():
    util::NumericTypeWrapper<Action, value_t>(NULL_ACTION)
{}

Action::Action(value_t val):
    util::NumericTypeWrapper<Action, value_t>(val)
{}

const char* Action::action_string(const Action& a) {
    return action_strings[a-NULL_ACTION];
}

const char* Action::action_string() const {
    return action_string(*this);
}

Action Action::random_action() {
    return min_action + rand()%(max_action-min_action);
}

std::ostream& operator<<(std::ostream &out, const Action& a) {
    out << a.action_string();
    return out;
}

const char* Action::action_strings[END_ACTION] = {
    "NULL_ACTION",
    "UP   ",
    "DOWN ",
    "LEFT ",
    "RIGHT",
    "STAY "
};

ActionIt::ActionIt() {}

ActionIt::ActionIt(const Action& a):
    Action(a),
    util::InvalidAdapter<ActionIt>(false)
{
    check_for_invalid();
}

ActionIt & ActionIt::operator++() {
    ++value;
    check_for_invalid();
    return *this;
}

ActionIt & ActionIt::operator--() {
    --value;
    check_for_invalid();
    return *this;
}

ActionIt & operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

ActionIt & operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
}

const ActionIt ActionIt::first() {
    return ActionIt(min_action);
}

const ActionIt ActionIt::last() {
    return ActionIt(max_action);
}

void ActionIt::check_for_invalid() {
    if( value<min_action || value>max_action ) {
        this->invalidate();
    }
}
