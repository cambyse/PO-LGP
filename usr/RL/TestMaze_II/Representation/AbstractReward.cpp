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

void AbstractReward::set_type(REWARD_TYPE t) {
    reward_type = t;
}
