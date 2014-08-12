#ifndef BUTTONOBSERVATION_H_
#define BUTTONOBSERVATION_H_

#include "../Representation/AbstractObservation.h"

class ButtonObservation: public AbstractObservation {
    //----typedefs/classes----//
    //          NONE          //

    //----members----//
    //          NONE          //

    //----methods----//
public:
    ButtonObservation();
    virtual ~ButtonObservation() override = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(ButtonObservation);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractObservation &other) const override;
    virtual bool operator<(const AbstractObservation &other) const override;
    virtual const std::string print() const override;
};

#endif /* BUTTONOBSERVATION_H_ */
