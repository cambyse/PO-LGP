#include "State.h"

State::State(value_t val):
    util::NumericTypeWrapper<State, unsigned long long int>(val) {}

StateIt::StateIt(const State& s):
    State(s),
    util::InvalidAdapter<StateIt>(false)
{}

StateIt & StateIt::operator++() {
    ++value;
    check_for_invalid();
    return *this;
}

StateIt & StateIt::operator--() {
    --value;
    check_for_invalid();
    return *this;
}

void StateIt::check_for_invalid() {
    if( value<min_state || value>max_state ) {
        this->invalidate();
    }
}
