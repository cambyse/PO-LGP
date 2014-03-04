#include "AbstractObservation.h"

#include "../util/debug.h"

using std::string;

AbstractObservation::AbstractObservation(OBSERVATION_TYPE t):
    observation_type(t)
{}

AbstractObservation::ptr_t AbstractObservation::next() const {
    return ptr_t(new AbstractObservation());
}

bool AbstractObservation::operator!=(const AbstractObservation& other) const {
    return this->observation_type!=other.observation_type;
}

bool AbstractObservation::operator<(const AbstractObservation& other) const {
    DEBUG_WARNING("Using comparison operator of abstract class");
    return this->observation_type<other.observation_type;
}

const string AbstractObservation::print() const {
    return string("AbstractObservation()");
}

AbstractObservation::OBSERVATION_TYPE AbstractObservation::get_type() const {
    return observation_type;
}

void AbstractObservation::set_type(OBSERVATION_TYPE t) {
    observation_type = t;
}
