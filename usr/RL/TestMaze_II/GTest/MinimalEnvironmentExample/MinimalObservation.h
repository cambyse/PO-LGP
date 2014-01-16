#ifndef MINIMALOBSERVATION_H_
#define MINIMALOBSERVATION_H_

#include "../../AbstractObservation.h"


class MinimalObservation: public AbstractObservation {
public:
    enum OBSERVATION { RED, GREEN } observation;
    MinimalObservation(OBSERVATION o = RED);
    virtual ~MinimalObservation() override = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(MinimalObservation);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractObservation &other) const override;
    virtual bool operator<(const AbstractObservation &other) const override;
    virtual const std::string print() const override;
};

#include "../../debug_exclude.h"

#endif /* MINIMALOBSERVATION_H_ */
