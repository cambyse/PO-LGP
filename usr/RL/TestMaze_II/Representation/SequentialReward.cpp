#include "SequentialReward.h"

#include "Config.h"

#include <math.h>

using util::INVALID;

const SequentialReward::value_t SequentialReward::min_reward = 0;
const SequentialReward::value_t SequentialReward::max_reward = 8;
const SequentialReward::value_t SequentialReward::reward_increment = 1;
const unsigned long SequentialReward::reward_n = floor((SequentialReward::max_reward - SequentialReward::min_reward)/SequentialReward::reward_increment) + 1;
const char* SequentialReward::type_string = "SequentialReward";

SequentialReward::SequentialReward(value_t val):
    util::NumericTypeWrapper<SequentialReward, value_t>(val) {}

std::ostream& operator<<(std::ostream &out, const SequentialReward& r) {
    int width = r.add_width();
    for(int i=0; i<width; ++i) {
        out << " ";
    }
    out << r.value;
    return out;
}

int SequentialReward::add_width() const {
    int max_width = 1+floor(log10(max_reward));
    int this_width = ( value>0 ) ? 1+floor(log10(value)) : 1;
    return max_width-this_width;
}

const SequentialRewardIt::All SequentialRewardIt::all = SequentialRewardIt::All();

SequentialRewardIt::SequentialRewardIt() {}

SequentialRewardIt::SequentialRewardIt(const SequentialReward& r):
    SequentialReward(r),
    util::InvalidAdapter<SequentialRewardIt>(false)
{
    check_for_invalid();
}

SequentialRewardIt & SequentialRewardIt::operator++() {
    value+=reward_increment;
    check_for_invalid();
    return *this;
}

SequentialRewardIt & SequentialRewardIt::operator--() {
    value-=reward_increment;
    check_for_invalid();
    return *this;
}

SequentialRewardIt & SequentialRewardIt::operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

SequentialRewardIt & SequentialRewardIt::operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
}

const SequentialRewardIt SequentialRewardIt::first() {
    return SequentialRewardIt(min_reward);
}

const SequentialRewardIt SequentialRewardIt::last() {
    return SequentialRewardIt(max_reward);
}

void SequentialRewardIt::check_for_invalid() {
    if( value<min_reward ||
        value>max_reward ||
        fmod(value,reward_increment)!=fmod(min_reward,reward_increment) ) {
        this->invalidate();
    }
}
