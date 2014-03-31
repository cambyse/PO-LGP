#ifndef ABSTRACTREWARD_H_
#define ABSTRACTREWARD_H_

#include "../util/util.h"
#include "../util/debug.h"

/** \brief Base class for all rewards. */
class AbstractReward: public util::AbstractIteratableSpace<AbstractReward> {
public:
    typedef double value_t;
    enum class REWARD_TYPE { NONE, MINIMAL, LISTED_REWARD };
    AbstractReward(REWARD_TYPE t = REWARD_TYPE::NONE);
    virtual ~AbstractReward() override {}
    ABSTRACT_ITERATABLE_SPACE_BEGIN(AbstractReward);
    virtual ptr_t next() const override;
    ABSTRACT_ITERATABLE_SPACE_NE(AbstractReward);
    virtual bool operator!=(const AbstractReward& other) const;
    ABSTRACT_ITERATABLE_SPACE_LT(AbstractReward);
    virtual bool operator<(const AbstractReward& other) const;
    virtual const std::string print() const;
    friend std::ostream& operator<<(std::ostream& out, const AbstractReward& a) {
        return out << a.print();
    }
    virtual REWARD_TYPE get_type() const;

    virtual value_t get_value() const;
    virtual value_t min_reward() const;
    virtual value_t max_reward() const;

protected:
    virtual void set_type(REWARD_TYPE t);
private:
    REWARD_TYPE reward_type;
};

#include "../util/debug_exclude.h"

#endif /* ABSTRACTREWARD_H_ */
