#ifndef MINIMALOBSERVATION_H_
#define MINIMALOBSERVATION_H_

#include "../../AbstractObservation.h"

class MinimalObservation: public AbstractObservation {
public:
    enum OBSERVATION { ONE, TWO, NONE } observation;
    MazeObservation(OBSERVATION o = NONE) {
        observation = o;
        set_type(OBSERVATION_TYPE::MINIMAL);
    }
    virtual ~MinimalObservation() = default;
    virtual Iterator begin() const override {
        return Iterator(new MazeObservation(ONE));
    }
    virtual ptr_t next() const override {
        if(observation==ONE) return ptr_t(new MinimalObservation(TWO));
        return end();
    }
    virtual bool operator!=(const AbstractObservation &other) const override {
        return observation!=other.observation;
    }
    virtual bool operator<(const AbstractObservation &other) const override {
        return observation<other.observation;
    }
    virtual const char * print() const override {
        if(observation==ONE) return std::string("MinimalObservation(ONE)").c_str();
        if(observation==TWO) return std::string("MinimalObservation(TWO)").c_str();
        return std::string("MinimalObservation(NONE)").c_str();
    }
    inline virtual const std::string space_descriptor() const override { return "MinimalObservation"; }
};

#endif /* MINIMALOBSERVATION_H_ */
