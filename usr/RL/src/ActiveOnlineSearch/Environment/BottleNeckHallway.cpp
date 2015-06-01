#include "BottleNeckHallway.h"

#include <util/util.h>
#include <util/debug.h>

using namespace util;

BottleNeckHallway::BottleNeckHallway(int length,
                                     int action_n,
                                     double min_prob,
                                     double max_prob):
    Environment(util::range_vector(action_n), util::range_vector(length)),
    length(length),
    action_n(action_n),
    min_prob(min_prob),
    max_prob(max_prob)
{
    DEBUG_EXPECT(0,length>0);
    DEBUG_EXPECT(0,action_n>0);
    DEBUG_EXPECT(0,min_prob>=0 && min_prob<=1);
    DEBUG_EXPECT(0,max_prob>=0 && max_prob<=1);
}

BottleNeckHallway::state_reward_pair_t BottleNeckHallway::transition(const state_t & state,
                                                                     const action_t & action) const {
    if(state>=length-1) {
        // in terminal state (or beyond)
        return state_reward_pair_t(state,0);
    } else if(state<length-2) {
        // before bottle-neck
        return state_reward_pair_t(state+1,0);
    } else {
        // at bottle-neck
        DEBUG_EXPECT(0,state==length-2);
        double prob = min_prob + (max_prob-min_prob)*action/(action_list.size()-1);
        if(drand48()<prob) {
            return state_reward_pair_t(state+1,1);
        } else {
            return state_reward_pair_t(state,0);
        }
    }
}
