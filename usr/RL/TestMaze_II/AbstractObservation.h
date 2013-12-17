#ifndef ABSTRACTOBSERVATION_H_
#define ABSTRACTOBSERVATION_H_

#include "util.h"

class AbstractObservation: public util::AbstractIteratableSpace<AbstractObservation> {
public:
    enum class OBSERVATION_TYPE { NONE, MINIMAL, MAZE_OBSERVATION };
    AbstractObservation(OBSERVATION_TYPE t = OBSERVATION_TYPE::NONE);
    ABSTRACT_ITERATABLE_SPACE_BEGIN(AbstractObservation);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractIteratableSpace& other) const override;
    virtual bool operator!=(const AbstractObservation& other) const;
    virtual bool operator<(const AbstractIteratableSpace& other) const override;
    virtual bool operator<(const AbstractObservation& other) const;
    virtual const std::string print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractObservation& a) {
        return out << a.print();
    }
    virtual OBSERVATION_TYPE get_type() const;
protected:
    virtual void set_type(OBSERVATION_TYPE t);
private:
    OBSERVATION_TYPE observation_type;
};

#endif /* ABSTRACTOBSERVATION_H_ */
