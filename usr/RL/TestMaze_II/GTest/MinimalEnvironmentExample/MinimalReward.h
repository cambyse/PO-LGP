#ifndef MINIMALREWARD_H_
#define MINIMALREWARD_H_

#include "../../AbstractReward.h"

class MinimalReward: public AbstractReward {
public:
    enum REWARD { NO_REWARD, SOME_REWARD } reward;
    MinimalReward(REWARD o = NO_REWARD);
    virtual ~MinimalReward() override = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(MinimalReward);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractReward &other) const override;
    virtual bool operator<(const AbstractReward &other) const override;
    virtual const std::string print() const override;
};

#include "../../debug_exclude.h"

#endif /* MINIMALREWARD_H_ */
