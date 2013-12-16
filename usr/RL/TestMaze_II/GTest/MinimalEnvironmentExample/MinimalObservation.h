#ifndef MINIMALOBSERVATION_H_
#define MINIMALOBSERVATION_H_

#include "../../AbstractObservation.h"

#include "../../debug.h"

class MinimalObservation: public AbstractObservation {
public:
    enum OBSERVATION { RED, GREEN, NONE } observation;
    MinimalObservation(OBSERVATION o = NONE) {
        observation = o;
        set_type(OBSERVATION_TYPE::MINIMAL);
    }
    virtual ~MinimalObservation() = default;
    virtual Iterator begin() const override {
        return Iterator(new MinimalObservation(RED));
    }
    virtual ptr_t next() const override {
        if(observation==RED) {
            return ptr_t(new MinimalObservation(GREEN));
        } else {
            return ptr_t(new MinimalObservation());
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
    virtual const char * print() const override {
        if(observation==RED) return std::string("MinimalObservation(RED)").c_str();
        if(observation==GREEN) return std::string("MinimalObservation(GREEN)").c_str();
        return std::string("MinimalObservation(NONE)").c_str();
    }
    inline virtual const std::string space_descriptor() const override { return "MinimalObservation"; }
};

#include "../../debug_exclude.h"

#endif /* MINIMALOBSERVATION_H_ */
