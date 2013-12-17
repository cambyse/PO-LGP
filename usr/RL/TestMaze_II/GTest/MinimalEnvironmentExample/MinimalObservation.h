#ifndef MINIMALOBSERVATION_H_
#define MINIMALOBSERVATION_H_

#include "../../AbstractObservation.h"

#include "../../debug.h"

class MinimalObservation: public AbstractObservation {
public:
    enum OBSERVATION { RED, GREEN } observation;
    MinimalObservation(OBSERVATION o = RED) {
        observation = o;
        set_type(OBSERVATION_TYPE::MINIMAL);
    }
    virtual ~MinimalObservation() = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(MinimalObservation);
    virtual ptr_t next() const override {
        if(observation==RED) {
            return ptr_t(new MinimalObservation(GREEN));
        } else {
            return ptr_t(new AbstractObservation());
        }
    }
    virtual bool operator!=(const AbstractObservation &other) const override {
        if(this->get_type()!=other.get_type()) {
            return true;
        } else {
            auto minimal_observation = dynamic_cast<const MinimalObservation *>(&other);
            if(minimal_observation==nullptr) {
                DEBUG_ERROR("Dynamic cast failed");
                return true;
            } else {
                return this->observation!=minimal_observation->observation;
            }
        }
    }
    virtual bool operator<(const AbstractObservation &other) const override {
        if(this->get_type()<other.get_type()) {
            return true;
        } else {
            auto minimal_observation = dynamic_cast<const MinimalObservation *>(&other);
            if(minimal_observation==nullptr) {
                DEBUG_ERROR("Dynamic cast failed");
                return true;
            } else {
                return this->observation<minimal_observation->observation;
            }
        }
    }
    virtual const std::string print() const override {
        if(observation==RED) return std::string("MinimalObservation(RED)");
        if(observation==GREEN) return std::string("MinimalObservation(GREEN)");
        return std::string("MinimalObservation(NONE)");
    }
};

#include "../../debug_exclude.h"

#endif /* MINIMALOBSERVATION_H_ */
