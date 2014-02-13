#include "Observation.h"

#include "../Config.h"

using util::INVALID;

const Observation::value_t Observation::min_observation = 0;
const Observation::value_t Observation::max_observation = Config::maze_x_size*Config::maze_y_size - 1;
const unsigned long Observation::observation_n = Observation::max_observation - Observation::min_observation + 1;

Observation::Observation(value_t val):
    util::NumericTypeWrapper<Observation, value_t>(val) {}

std::ostream& operator<<(std::ostream &out, const Observation& s) {
    int width = s.add_width();
    for(int i=0; i<width; ++i) {
        out << " ";
    }
    out << s.value;
    return out;
}

Observation Observation::random_observation() {
    return min_observation + rand()%(max_observation-min_observation);
}

int Observation::add_width() const {
    int max_width = 1+floor(log10(max_observation));
    int this_width = ( value>0 ) ? 1+floor(log10(value)) : 1;
    return max_width-this_width;
}

const ObservationIt::All ObservationIt::all = ObservationIt::All();

ObservationIt::ObservationIt() {}

ObservationIt::ObservationIt(const Observation& s):
    Observation(s),
    util::InvalidAdapter<ObservationIt>(false)
{}

ObservationIt & ObservationIt::operator++() {
    ++value;
    check_for_invalid();
    return *this;
}

ObservationIt & ObservationIt::operator--() {
    --value;
    check_for_invalid();
    return *this;
}

ObservationIt & ObservationIt::operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

ObservationIt & ObservationIt::operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
}

const ObservationIt ObservationIt::first() {
    return ObservationIt(min_observation);
}

const ObservationIt ObservationIt::last() {
    return ObservationIt(max_observation);
}

void ObservationIt::check_for_invalid() {
    if( value<min_observation || value>max_observation ) {
        this->invalidate();
    }
}
