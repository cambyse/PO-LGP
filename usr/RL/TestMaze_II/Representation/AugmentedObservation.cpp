#include "AugmentedObservation.h"

#include "../Config.h"

using util::INVALID;

const AugmentedObservation::value_t AugmentedObservation::min_observation = 0;
const AugmentedObservation::value_t AugmentedObservation::max_observation = Config::maze_x_size*Config::maze_y_size - 1;
const unsigned long AugmentedObservation::observation_n = AugmentedObservation::max_observation - AugmentedObservation::min_observation + 1;

AugmentedObservation::AugmentedObservation(value_t val):
    util::NumericTypeWrapper<AugmentedObservation, value_t>(val) {}

std::ostream& operator<<(std::ostream &out, const AugmentedObservation& s) {
    int width = s.add_width();
    for(int i=0; i<width; ++i) {
        out << " ";
    }
    out << s.value;
    return out;
}

AugmentedObservation AugmentedObservation::random_observation() {
    return min_observation + rand()%(max_observation-min_observation);
}

int AugmentedObservation::add_width() const {
    int max_width = 1+floor(log10(max_observation));
    int this_width = ( value>0 ) ? 1+floor(log10(value)) : 1;
    return max_width-this_width;
}

const AugmentedObservationIt::All AugmentedObservationIt::all = AugmentedObservationIt::All();

AugmentedObservationIt::AugmentedObservationIt() {}

AugmentedObservationIt::AugmentedObservationIt(const AugmentedObservation& s):
    AugmentedObservation(s),
    util::InvalidAdapter<AugmentedObservationIt>(false)
{}

AugmentedObservationIt & AugmentedObservationIt::operator++() {
    ++value;
    check_for_invalid();
    return *this;
}

AugmentedObservationIt & AugmentedObservationIt::operator--() {
    --value;
    check_for_invalid();
    return *this;
}

AugmentedObservationIt & AugmentedObservationIt::operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

AugmentedObservationIt & AugmentedObservationIt::operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
}

const AugmentedObservationIt AugmentedObservationIt::first() {
    return AugmentedObservationIt(min_observation);
}

const AugmentedObservationIt AugmentedObservationIt::last() {
    return AugmentedObservationIt(max_observation);
}

void AugmentedObservationIt::check_for_invalid() {
    if( value<min_observation || value>max_observation ) {
        this->invalidate();
    }
}
