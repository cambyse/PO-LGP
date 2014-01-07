#include "CheeseMazeObservation.h"

#include "../../util/Macro.h"

#include "../../debug.h"

CheeseMazeObservation::CheeseMazeObservation(OBSERVATION o) {
    observation = o;
    set_type(OBSERVATION_TYPE::CHEESE_MAZE_OBSERVATION);
}

CheeseMazeObservation::ptr_t CheeseMazeObservation::next() const {
    OBSERVATION a = (OBSERVATION)((int)observation+1);
    if(a>=OBSERVATION::END) {
        return ptr_t(new AbstractObservation());
    } else {
        return ptr_t(new CheeseMazeObservation(a));
    }
}


bool CheeseMazeObservation::operator!=(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const CheeseMazeObservation *);
    return this->observation!=ptr->observation;
}

bool CheeseMazeObservation::operator<(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const CheeseMazeObservation *);
    return this->observation < ptr->observation;
}

const std::string CheeseMazeObservation::print() const {
    string ret("CheeseMazeObservation(");
    switch(observation) {
    case OBSERVATION::N:
        ret+="  N";
        break;
    case OBSERVATION::NE:
        ret+=" NE";
        break;
    case OBSERVATION::NS:
        ret+=" NS";
        break;
    case OBSERVATION::NW:
        ret+=" NW";
        break;
    case OBSERVATION::EW:
        ret+=" EW";
        break;
    case OBSERVATION::ESW:
        ret+="ESW";
        break;
    default:
        DEBUG_ERROR("Invalid observation");
        ret+="INVALID";
        break;
    }
    ret+=")";
    return ret;
}
