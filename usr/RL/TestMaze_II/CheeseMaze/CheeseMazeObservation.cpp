#include "CheeseMazeObservation.h"

#include "../util/debug.h"

using std::string;

#define USE_UTF8

CheeseMazeObservation::CheeseMazeObservation(OBSERVATION o) {
    observation = o;
    set_type(OBSERVATION_TYPE::CHEESE_MAZE_OBSERVATION);
}

CheeseMazeObservation::CheeseMazeObservation(const char * c) {
    if(!strcmp(c,"N")) {
        observation = OBSERVATION::N;
    } else if(!strcmp(c,"NE")) {
        observation = OBSERVATION::NE;
    } else if(!strcmp(c,"NS")) {
        observation = OBSERVATION::NS;
    } else if(!strcmp(c,"NW")) {
        observation = OBSERVATION::NW;
    } else if(!strcmp(c,"EW")) {
        observation = OBSERVATION::EW;
    } else if(!strcmp(c,"ESW")) {
        observation = OBSERVATION::ESW;
    } else {
        DEBUG_ERROR("Not valid observation ('" << c << "'");
        observation = OBSERVATION::N;
    }
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
#ifdef USE_UTF8
    case OBSERVATION::N:
        ret+=" ̅ ̅";
        break;
    case OBSERVATION::NE:
        ret+="╶┐";
        break;
    case OBSERVATION::NS:
        ret+=" ̲̅ ̲̅";
        break;
    case OBSERVATION::NW:
        ret+="┌╴";
        break;
    case OBSERVATION::EW:
        ret+="||";
        break;
    case OBSERVATION::ESW:
        ret+="└┘";
        break;
#else
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
#endif
    default:
        DEBUG_ERROR("Invalid observation");
        ret+="INVALID";
        break;
    }
    ret+=")";
    return ret;
}
