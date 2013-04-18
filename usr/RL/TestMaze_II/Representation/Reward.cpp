#include "Reward.h"

#include <math.h>

const Reward::value_t Reward::min_reward = 0;
const Reward::value_t Reward::max_reward = 10;
const Reward::value_t Reward::reward_increment = 1;

Reward::Reward(value_t val):
    util::NumericTypeWrapper<Reward, value_t>(val) {}

std::ostream& operator<<(std::ostream &out, const Reward& r) {
    int width = r.add_width();
    for(int i=0; i<width; ++i) {
        out << " ";
    }
    out << r.value;
    return out;
}

int Reward::add_width() const {
    int max_width = 1+floor(log10(max_reward));
    int this_width = ( value>0 ) ? 1+floor(log10(value)) : 1;
    return max_width-this_width;
}


RewardIt::RewardIt() {}

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

RewardIt & operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

RewardIt & operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
}

const RewardIt RewardIt::first() {
    return RewardIt(min_reward);
}

const RewardIt RewardIt::last() {
    return RewardIt(max_reward);
}

void RewardIt::check_for_invalid() {
    if( value<min_reward ||
        value>max_reward ||
        std::fmod(value,reward_increment)!=std::fmod(min_reward,reward_increment) ) {
        this->invalidate();
    }
}
