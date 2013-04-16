#include "Reward.h"

#include <math.h>

const Reward::value_t Reward::min_reward = 0;
const Reward::value_t Reward::max_reward = 10;
const Reward::value_t Reward::reward_increment = 1;

Reward::Reward(value_t val):
    util::NumericTypeWrapper<Reward, value_t>(val) {}

RewardIt::RewardIt(const Reward& r):
    Reward(r),
    util::InvalidAdapter<RewardIt>(false)
{
    check_for_invalid();
}

RewardIt & RewardIt::operator++() {
    value+=reward_increment;
    check_for_invalid();
    return *this;
}

RewardIt & RewardIt::operator--() {
    value-=reward_increment;
    check_for_invalid();
    return *this;
}

void RewardIt::check_for_invalid() {
    if( value<min_reward ||
        value>max_reward ||
        std::fmod(value,reward_increment)!=std::fmod(min_reward,reward_increment) ) {
        this->invalidate();
    }
}
