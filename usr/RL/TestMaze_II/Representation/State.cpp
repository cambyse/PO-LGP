#include "State.h"

#include "Config.h"

USE_CONFIG_CONSTS;

using util::INVALID;

const State::value_t State::min_state = 0;
const State::value_t State::max_state = maze_x_size*maze_y_size - 1;
const unsigned long State::state_n = State::max_state - State::min_state + 1;

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

State State::random_state() {
    return min_state + rand()%(max_state-min_state);
}

int State::add_width() const {
    int max_width = 1+floor(log10(max_state));
    int this_width = ( value>0 ) ? 1+floor(log10(value)) : 1;
    return max_width-this_width;
}

const StateIt::All StateIt::all = StateIt::All();

StateIt::StateIt() {}

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

StateIt & StateIt::operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

StateIt & StateIt::operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
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
