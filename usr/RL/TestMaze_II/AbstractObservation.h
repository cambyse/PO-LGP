#ifndef ABSTRACTOBSERVATION_H_
#define ABSTRACTOBSERVATION_H_

#include "util.h"

class AbstractObservation: public util::AbstractIteratableSpace {
public:
    enum class OBSERVATION_TYPE { NONE, MAZE_OBSERVATION };
    AbstractObservation(OBSERVATION_TYPE t = OBSERVATION_TYPE::NONE);
    virtual Iterator begin() const override;
    virtual Iterator end() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractIteratableSpace& other) const override;
    virtual bool operator!=(const AbstractObservation& other) const;
    virtual std::string print() const override;
    friend std::ostream& operator<<(std::ostream& out, const AbstractObservation& a);
    virtual OBSERVATION_TYPE get_type() const;
protected:
    virtual void set_type(OBSERVATION_TYPE t);
private:
    OBSERVATION_TYPE observation_type;
};

#endif /* ABSTRACTOBSERVATION_H_ */
