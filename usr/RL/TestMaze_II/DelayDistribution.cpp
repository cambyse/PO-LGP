#include "DelayDistribution.h"

#define DEBUG_STRING "DelayDist: "
#define DEBUG_LEVEL 1
#include "debug.h"

using util::INVALID;

DelayDistribution::probability_t DelayDistribution::get_delay_probability(
    const state_t& s1,
    const state_t& s2,
    const idx_t& delay
    ) {

    probability_t prob = 0;
    size_t normalization = 0;
    if(instance_data!=nullptr) {
        const_instanceIt_t insIt_1 = instance_data->const_first();
        const_instanceIt_t insIt_2 = insIt_1;
        if(delay<0) {
            DEBUG_OUT(1,"Warning: Using negative delay");
            insIt_1 -= delay;
        } else {
            insIt_2 += delay;
        }
        while(insIt_1!=INVALID && insIt_2!=INVALID) {
            if(insIt_1->state==s1) {
                ++normalization;
                if(insIt_2->state==s2) {
                    prob += 1;
                }
            }
            ++insIt_1;
            ++insIt_2;
        }
    }

    if(normalization!=0) {
        return prob/normalization;
    } else {
        DEBUG_OUT(1,"Warning: Not enough data to determine probability");
        return 0;
    }
}
