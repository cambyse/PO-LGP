#include "EnumeratedReward.h"

#include <math.h>

#define DEBUG_LEVEL 0
#define DEBUG_STRING "EnumeratedReward: "
#include "../debug.h"

using util::INVALID;
using std::vector;

const vector<EnumeratedReward::value_t> EnumeratedReward::reward_vector = {
    0,
    1
};
const EnumeratedReward::value_t EnumeratedReward::min_reward = EnumeratedReward::reward_vector.front();
const EnumeratedReward::value_t EnumeratedReward::max_reward = EnumeratedReward::reward_vector.back();
const unsigned long EnumeratedReward::reward_n = EnumeratedReward::reward_vector.size();
const char* EnumeratedReward::type_string = "EnumeratedReward";

EnumeratedReward::EnumeratedReward(value_t val):
    util::NumericTypeWrapper<EnumeratedReward, value_t>(val) {}

std::ostream& operator<<(std::ostream &out, const EnumeratedReward& r) {
    int width = r.add_width();
    for(int i=0; i<width; ++i) {
        out << " ";
    }
    out << r.value;
    return out;
}

unsigned long EnumeratedReward::index() const {
    for(unsigned long r_idx=0; r_idx<reward_n; ++r_idx) {
        if(this->value==reward_vector[r_idx]) {
            return r_idx;
        }
    }
    DEBUG_OUT(0,"Error: Could not find reward index");
    return 0;
}

int EnumeratedReward::add_width() const {
    int max_width = 1+floor(log10(max_reward));
    int this_width = ( value>0 ) ? 1+floor(log10(value)) : 1;
    return max_width-this_width;
}

const EnumeratedRewardIt::All EnumeratedRewardIt::all = EnumeratedRewardIt::All();

EnumeratedRewardIt::EnumeratedRewardIt() {}

// EnumeratedRewardIt::EnumeratedRewardIt(const EnumeratedReward& r):
//     EnumeratedReward(r),
//     util::InvalidAdapter<EnumeratedRewardIt>(false)
// {
//     check_for_invalid();
// }

EnumeratedRewardIt & EnumeratedRewardIt::operator++() {
    ++reward_index;
    value=reward_vector[reward_index];
    check_for_invalid();
    return *this;
}

EnumeratedRewardIt & EnumeratedRewardIt::operator--() {
    --reward_index;
    value=reward_vector[reward_index];
    check_for_invalid();
    return *this;
}

EnumeratedRewardIt & EnumeratedRewardIt::operator+=(const int& c) {
    if(c<0) {
        return (*this) -= -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            ++(*this);
        }
        return (*this);
    }
}

EnumeratedRewardIt & EnumeratedRewardIt::operator-=(const int& c) {
    if(c<0) {
        return (*this) += -c;
    } else {
        for(int i=0; i<c && (*this)!=INVALID; ++i) {
            --(*this);
        }
        return (*this);
    }
}

const EnumeratedRewardIt EnumeratedRewardIt::first() {
    return EnumeratedRewardIt(0);
}

const EnumeratedRewardIt EnumeratedRewardIt::last() {
    return EnumeratedRewardIt(reward_n-1);
}

EnumeratedRewardIt::EnumeratedRewardIt(const int& idx):
    EnumeratedReward(reward_vector[idx]),
    util::InvalidAdapter<EnumeratedRewardIt>(false),
    reward_index(idx)
{}

void EnumeratedRewardIt::check_for_invalid() {
    if(reward_index>=reward_n) {
        this->invalidate();
    }
}
