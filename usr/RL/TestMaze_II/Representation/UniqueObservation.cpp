#include "UniqueObservation.h"

#include <string>

#include "../util/debug.h"

using std::string;

UniqueObservation::UniqueObservation() {
    set_type(OBSERVATION_TYPE::UNIQUE_OBSERVATION);
}

UniqueObservation::ptr_t UniqueObservation::next() const {
    return ptr_t(new AbstractObservation());
}


bool UniqueObservation::operator!=(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const UniqueObservation *);
    return false;
}

bool UniqueObservation::operator<(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const UniqueObservation *);
    return false;
}

const std::string UniqueObservation::print() const {
    string ret;
    if(!print_short_name) {
        ret += "UniqueObservation";
    } else {
        ret += "--";
    }
    return ret;
}

