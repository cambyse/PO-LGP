#ifndef UNIQUEOBSERVATION_H_
#define UNIQUEOBSERVATION_H_

#include "../Representation/AbstractObservation.h"

class UniqueObservation: public AbstractObservation {
    //----typedefs/classes----//
    //          NONE          //

    //----members----//
    //          NONE          //

    //----methods----//
public:
    UniqueObservation();
    virtual ~UniqueObservation() override = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(UniqueObservation);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractObservation &other) const override;
    virtual bool operator<(const AbstractObservation &other) const override;
    virtual const std::string print() const override;
};

#endif /* UNIQUEOBSERVATION_H_ */
