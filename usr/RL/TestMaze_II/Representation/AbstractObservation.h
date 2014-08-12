#ifndef ABSTRACTOBSERVATION_H_
#define ABSTRACTOBSERVATION_H_

#include "../util/util.h"
#include "../util/debug.h"

/** \brief Base class for all observations. */
class AbstractObservation: public util::AbstractIteratableSpace<AbstractObservation> {
public:
    enum class OBSERVATION_TYPE { NONE
            ,MINIMAL
            ,MAZE_OBSERVATION
            ,CHEESE_MAZE_OBSERVATION
            ,BUTTON_OBSERVATION
            };
    AbstractObservation(OBSERVATION_TYPE t = OBSERVATION_TYPE::NONE);
    virtual ~AbstractObservation() override {}
    ABSTRACT_ITERATABLE_SPACE_BEGIN(AbstractObservation);
    virtual ptr_t next() const override;
    ABSTRACT_ITERATABLE_SPACE_NE(AbstractObservation);
    virtual bool operator!=(const AbstractObservation& other) const;
    ABSTRACT_ITERATABLE_SPACE_LT(AbstractObservation);
    virtual bool operator<(const AbstractObservation& other) const;
    virtual const std::string print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractObservation& a) {
        return out << a.print();
    }
    virtual OBSERVATION_TYPE get_type() const;
protected:
    bool print_short_name = true;
    virtual void set_type(OBSERVATION_TYPE t);
private:
    OBSERVATION_TYPE observation_type;
};

#include "../util/debug_exclude.h"

#endif /* ABSTRACTOBSERVATION_H_ */
