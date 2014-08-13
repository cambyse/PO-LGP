#include "ButtonObservation.h"

#include <string>

#include "../util/debug.h"

using std::string;

ButtonObservation::ButtonObservation() {
    set_type(OBSERVATION_TYPE::BUTTON_OBSERVATION);
}

ButtonObservation::ptr_t ButtonObservation::next() const {
    return ptr_t(new AbstractObservation());
}


bool ButtonObservation::operator!=(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const ButtonObservation *);
    return false;
}

bool ButtonObservation::operator<(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const ButtonObservation *);
    return false;
}

const std::string ButtonObservation::print() const {
    string ret;
    if(!print_short_name) {
        ret += "ButtonObservation";
    } else {
        ret += "--";
    }
    return ret;
}

