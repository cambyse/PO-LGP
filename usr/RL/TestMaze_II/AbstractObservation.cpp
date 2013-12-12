#include "AbstractObservation.h"

#include "debug.h"

AbstractObservation::AbstractObservation(OBSERVATION_TYPE t):
    observation_type(t)
{}

AbstractObservation::Iterator AbstractObservation::begin() const {
    return Iterator(ptr_t(new AbstractObservation()));
}

AbstractObservation::ptr_t AbstractObservation::next() const {
    return ptr_t(new AbstractObservation());
}

bool AbstractObservation::operator!=(const AbstractIteratableSpace& other) const {
    auto abstract_observation = dynamic_cast<const AbstractObservation *>(&other);
    if(abstract_observation==nullptr) {
        DEBUG_ERROR("Dynamic cast failed");
        return true;
    } else {
        return *this!=*abstract_observation;
    }
}

bool AbstractObservation::operator!=(const AbstractObservation& other) const {
    return this->observation_type!=other.observation_type;
}

bool AbstractObservation::operator<(const AbstractIteratableSpace& other) const {
    auto abstract_observation = dynamic_cast<const AbstractObservation *>(&other);
    if(abstract_observation==nullptr) {
        DEBUG_ERROR("Dynamic cast failed");
        return true;
    } else {
        return *this<*abstract_observation;
    }
}

bool AbstractObservation::operator<(const AbstractObservation& other) const {
    return this->observation_type<other.observation_type;
}

const char * AbstractObservation::print() const {
    return "AbstractObservation()";
}

AbstractObservation::OBSERVATION_TYPE AbstractObservation::get_type() const {
    return observation_type;
}

void AbstractObservation::set_type(OBSERVATION_TYPE t) {
    observation_type = t;
}
