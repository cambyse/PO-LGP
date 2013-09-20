#include "AugmentedState.h"

#include "Config.h"

using util::INVALID;

const AugmentedState::value_t AugmentedState::min_state = 0;
const AugmentedState::value_t AugmentedState::max_state = Config::maze_x_size*Config::maze_y_size - 1;
const unsigned long AugmentedState::state_n = AugmentedState::max_state - AugmentedState::min_state + 1;

AugmentedState::AugmentedState(value_t val):
    util::NumericTypeWrapper<AugmentedState, value_t>(val) {}

std::ostream& operator<<(std::ostream &out, const AugmentedState& s) {
    int width = s.add_width();
    for(int i=0; i<width; ++i) {
        out << " ";
    }
    out << s.value;
    return out;
}

AugmentedState AugmentedState::random_state() {
    return min_state + rand()%(max_state-min_state);
}

int AugmentedState::add_width() const {
    int max_width = 1+floor(log10(max_state));
    int this_width = ( value>0 ) ? 1+floor(log10(value)) : 1;
    return max_width-this_width;
}

const AugmentedStateIt::All AugmentedStateIt::all = AugmentedStateIt::All();

AugmentedStateIt::AugmentedStateIt() {}

AugmentedStateIt::AugmentedStateIt(const AugmentedState& s):
    AugmentedState(s),
    util::InvalidAdapter<AugmentedStateIt>(false)
{}

AugmentedStateIt & AugmentedStateIt::operator++() {
    ++value;
    check_for_invalid();
    return *this;
}

AugmentedStateIt & AugmentedStateIt::operator--() {
    --value;
    check_for_invalid();
    return *this;
}

AugmentedStateIt & AugmentedStateIt::operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

AugmentedStateIt & AugmentedStateIt::operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
}

const AugmentedStateIt AugmentedStateIt::first() {
    return AugmentedStateIt(min_state);
}

const AugmentedStateIt AugmentedStateIt::last() {
    return AugmentedStateIt(max_state);
}

void AugmentedStateIt::check_for_invalid() {
    if( value<min_state || value>max_state ) {
        this->invalidate();
    }
}
