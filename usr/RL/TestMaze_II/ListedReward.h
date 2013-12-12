#ifndef LISTEDREWARD_H_
#define LISTEDREWARD_H_

#include "../AbstractReward.h"

#include <vector>

class ListedReward: public AbstractReward {
public:
    ListedReward(
        const std::vector<float> & list = std::vector<float>(),
        const uint & idx = 0
        );
    virtual Iterator begin() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractReward &other) const override;
    virtual bool operator<(const AbstractReward &other) const override;
    virtual const char * print() const override;
protected:
    virtual void set_type(REWARD_TYPE t) override;
    std::vector<float> reward_list;
    uint reward_index;
};

#endif /* LISTEDREWARD_H_ */
