#include "Action.h"

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

ActionIt::ActionIt() {
    *this = first();
}

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

const ActionIt ActionIt::first() {
    return ActionIt(NULL_ACTION+1);
}

const ActionIt ActionIt::last() {
    return ActionIt(END_ACTION-1);
}

void ActionIt::check_for_invalid() {
    if( value<=NULL_ACTION || value>=END_ACTION ) {
        this->invalidate();
    }
}
