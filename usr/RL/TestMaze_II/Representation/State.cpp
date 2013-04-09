#include "State.h"

State::State(value_t val):
    util::NumericTypeWrapper<State, unsigned long long int>(val) {}

StateIt::StateIt(const State& s): State(s) {}

StateIt & StateIt::operator++() {
    ++value;
    return *this;
}

StateIt & StateIt::operator--() {
    --value;
    return *this;
}

bool StateIt::is_valid() const {
    return ( value>=min_state && value<=max_state );
}
