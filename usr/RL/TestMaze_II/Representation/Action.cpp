#include "Action.h"

using util::INVALID;

const Action::value_t Action::min_action = Action::UP;
const Action::value_t Action::max_action = Action::STAY;
const unsigned long Action::action_n = Action::END_ACTION - Action::NULL_ACTION - 1;

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
    return min_action + rand()%(max_action - min_action + 1);
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

const ActionIt::All ActionIt::all = ActionIt::All();

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

ActionIt & ActionIt::operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

ActionIt & ActionIt::operator-=(const int& c) {
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
