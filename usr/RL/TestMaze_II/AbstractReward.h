#ifndef ABSTRACTREWARD_H_
#define ABSTRACTREWARD_H_

#include "util.h"

class AbstractReward: public util::AbstractIteratableSpace {
public:
    enum class REWARD_TYPE { NONE, LISTED_REWARD };
    AbstractReward(REWARD_TYPE t = REWARD_TYPE::NONE);
    virtual Iterator begin() const override;
    virtual Iterator end() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractIteratableSpace& other) const override;
    virtual bool operator!=(const AbstractReward& other) const;
    virtual std::string print() const override;
    friend std::ostream& operator<<(std::ostream& out, const AbstractReward& a);
    virtual REWARD_TYPE get_type() const;
protected:
    virtual void set_type(REWARD_TYPE t);
private:
    REWARD_TYPE reward_type;
};

#endif /* ABSTRACTREWARD_H_ */
