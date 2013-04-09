#include "Reward.h"

#include <math.h>

const Reward::value_t Reward::min_reward = 0;
const Reward::value_t Reward::max_reward = 10;
const Reward::value_t Reward::reward_increment = 1;

Reward::Reward(value_t val):
    util::NumericTypeWrapper<Reward, double>(val) {}

RewardIt::RewardIt(const Reward& r): Reward(r) {}

RewardIt & RewardIt::operator++() {
    value+=reward_increment;
    return *this;
}

RewardIt & RewardIt::operator--() {
    value-=reward_increment;
    return *this;
}

bool RewardIt::is_valid() const {
    return ( value>=min_reward &&
             value<=max_reward && 
             std::fmod(value,reward_increment)==std::fmod(min_reward,reward_increment) );
}
