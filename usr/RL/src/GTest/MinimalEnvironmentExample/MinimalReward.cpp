#include "MinimalReward.h"

#include "../../util/debug.h"

MinimalReward::MinimalReward(REWARD o) {
    reward = o;
    set_type(REWARD_TYPE::MINIMAL);
}

MinimalReward::ptr_t MinimalReward::next() const {
    if(reward==NO_REWARD) {
        return ptr_t(new MinimalReward(SOME_REWARD));
    } else {
        return ptr_t(new AbstractReward());
    }
}

bool MinimalReward::operator!=(const AbstractReward &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const MinimalReward *);
    return this->reward!=ptr->reward;
}

bool MinimalReward::operator<(const AbstractReward &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const MinimalReward *);
    return this->reward < ptr->reward;
}

const std::string MinimalReward::print() const {
    if(reward==NO_REWARD) return std::string("MinimalReward(NO_REWARD)");
    if(reward==SOME_REWARD) return std::string("MinimalReward(SOME_REWARD)");
    return std::string("MinimalReward(NONE)");
}
