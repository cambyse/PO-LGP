#ifndef ABSTRACTOBSERVATION_H_
#define ABSTRACTOBSERVATION_H_

#include "util.h"

class AbstractObservation: public util::AbstractIteratableSpace<AbstractObservation> {
public:
    enum class OBSERVATION_TYPE { NONE, MINIMAL, MAZE_OBSERVATION };
    AbstractObservation(OBSERVATION_TYPE t = OBSERVATION_TYPE::NONE);
    virtual Iterator begin() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractIteratableSpace& other) const override;
    virtual bool operator!=(const AbstractObservation& other) const;
    virtual bool operator<(const AbstractIteratableSpace& other) const override;
    virtual bool operator<(const AbstractObservation& other) const;
    virtual const char * print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractObservation& a) {
        return out << a.print();
    }
    virtual OBSERVATION_TYPE get_type() const;
    inline virtual const std::string space_descriptor() const override { return "AbstractObservation"; }
protected:
    virtual void set_type(OBSERVATION_TYPE t);
private:
    OBSERVATION_TYPE observation_type;
};

#endif /* ABSTRACTOBSERVATION_H_ */
