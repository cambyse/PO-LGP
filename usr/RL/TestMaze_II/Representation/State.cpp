#include "State.h"

State::State(value_t val):
    util::NumericTypeWrapper<State, value_t>(val) {}

std::ostream& operator<<(std::ostream &out, const State& s) {
    int width = s.add_width();
    for(int i=0; i<width; ++i) {
        out << " ";
    }
    out << s.value;
    return out;
}

int State::add_width() const {
    int max_width = 1+floor(log10(max_state));
    int this_width = ( value>0 ) ? 1+floor(log10(value)) : 1;
    return max_width-this_width;
}

StateIt::StateIt() {
    *this = first();
}

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

const StateIt StateIt::first() {
    return StateIt(min_state);
}

const StateIt StateIt::last() {
    return StateIt(max_state);
}

void StateIt::check_for_invalid() {
    if( value<min_state || value>max_state ) {
        this->invalidate();
    }
}
