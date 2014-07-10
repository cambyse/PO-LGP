#include "MinimalObservation.h"

#include "../../util/debug.h"

MinimalObservation::MinimalObservation(OBSERVATION o) {
    observation = o;
    set_type(OBSERVATION_TYPE::MINIMAL);
}

MinimalObservation::ptr_t MinimalObservation::next() const {
    if(observation==RED) {
        return ptr_t(new MinimalObservation(GREEN));
    } else {
        return ptr_t(new AbstractObservation());
    }
}


bool MinimalObservation::operator!=(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const MinimalObservation *);
    return this->observation!=ptr->observation;
}

bool MinimalObservation::operator<(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const MinimalObservation *);
    return this->observation < ptr->observation;
}

const std::string MinimalObservation::print() const {
    if(observation==RED) return std::string("MinimalObservation(RED)");
    if(observation==GREEN) return std::string("MinimalObservation(GREEN)");
    return std::string("MinimalObservation(NONE)");
}
