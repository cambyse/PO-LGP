#include "Action.h"

Action::Action(value_t val):
    util::NumericTypeWrapper<Action, unsigned long long int>(val) {}

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

ActionIt::ActionIt(const Action& a): Action(a) {}

ActionIt & ActionIt::operator++() {
    ++value;
    return *this;
}

ActionIt & ActionIt::operator--() {
    --value;
    return *this;
}

bool ActionIt::is_valid() const {
    return ( value>NULL_ACTION && value<END_ACTION );
}
