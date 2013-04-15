#include "Action.h"

Action::Action():
    util::NumericTypeWrapper<Action, unsigned long long int>(NULL_ACTION),
    util::InvalidAdapter<Action>(true)
{}

Action::Action(value_t val):
    util::NumericTypeWrapper<Action, unsigned long long int>(val),
    util::InvalidAdapter<Action>(false)
{}

const char* Action::action_string(const Action& a) {
    return action_strings[a-NULL_ACTION];
}

const char* Action::action_string() const {
    return action_string(*this);
}

const char* Action::action_strings[END_ACTION] = {
    "NULL_ACTION",
    "UP   ",
    "DOWN ",
    "LEFT ",
    "RIGHT",
    "STAY "
};

ActionIt::ActionIt(const Action& a): Action(a) {
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

void ActionIt::check_for_invalid() {
    if( value<=NULL_ACTION || value>=END_ACTION ) {
        this->invalidate();
    }
}
