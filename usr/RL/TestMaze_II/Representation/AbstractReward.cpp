#include "AbstractReward.h"

#include "../util/debug.h"

using std::string;

AbstractReward::AbstractReward(REWARD_TYPE t):
    reward_type(t)
{}

AbstractReward::ptr_t AbstractReward::next() const {
    return ptr_t(new AbstractReward());
}

bool AbstractReward::operator!=(const AbstractReward& other) const {
    return this->reward_type!=other.reward_type;
}

bool AbstractReward::operator<(const AbstractReward& other) const {
    DEBUG_WARNING("Using comparison operator of abstract class");
    return this->reward_type<other.reward_type;
}

const string AbstractReward::print() const {
    return string("AbstractReward()");
}

AbstractReward::REWARD_TYPE AbstractReward::get_type() const {
    return reward_type;
}

AbstractReward::value_t AbstractReward::get_value() const {
    DEBUG_ERROR("Getting value of " << *this);
    return 0;
}

AbstractReward::value_t AbstractReward::min_reward() const {
    DEBUG_ERROR("Getting min reward of " << *this);
    return 0;
}

AbstractReward::value_t AbstractReward::max_reward() const {
    DEBUG_ERROR("Getting max reward of " << *this);
    return 0;
}

void AbstractReward::set_type(REWARD_TYPE t) {
    reward_type = t;
}
