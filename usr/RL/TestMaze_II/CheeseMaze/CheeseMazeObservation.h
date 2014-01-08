#ifndef CHEESEMAZEOBSERVATION_H_
#define CHEESEMAZEOBSERVATION_H_

#include "../AbstractObservation.h"

class CheeseMazeObservation: public AbstractObservation {
public:
    enum OBSERVATION { N, NE, NS, NW, EW, ESW, END } observation;
    CheeseMazeObservation(OBSERVATION o = N);
    CheeseMazeObservation(const char * c);
    virtual ~CheeseMazeObservation() = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(CheeseMazeObservation);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractObservation &other) const override;
    virtual bool operator<(const AbstractObservation &other) const override;
    virtual const std::string print() const override;
};

#endif /* CHEESEMAZEOBSERVATION_H_ */
