#include "DynamicTightRope.h"

#include <util/util.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using util::Range;
using std::tuple;
using std::vector;

DynamicTightRope::DynamicTightRope(int n):
    Environment({0,1}, vector<state_t>(velocity_n*n)),
    action_names({"accelerate", "keep velocity", "decelerate"}),
    state_names(velocity_n*n) {
    for(int i : Range(n)) {
        for(int vel : Range(velocity_n)) {
            states[i+n*vel] = i+n*vel;
            state_names[i+n*vel] = QString("state %1, velocity %2").arg(i).arg(vel);
        }
    }
}

DynamicTightRope::state_reward_pair_t DynamicTightRope::sample(const state_t & s, const action_t & a) const {
    // return values
    state_t ss = 0;
    reward_t r = 0;
    // success probabilities for forward moves
    double max_success_rate = 1;
    double min_success_rate = 0.8;
    double max_success_rate_fast = 1;
    double min_success_rate_fast = 0.4;
    // position on the rope
    double t = ((double)s)/(states.size()-1); // in [0,1] linearly increasing
    t = pow(2*(t-0.5),2);                    // in [0,1] with minimum at 0.5
    t = 1-exp(-t/0.15);
    DEBUG_OUT(1,"Path = " << t << " (s = " << s << ")");
    // perform action
    // if(a==FORWARD) {
    //     DEBUG_OUT(1,"Rate = " << t*max_success_rate+(1-t)*min_success_rate);
    //     if(drand48()<t*max_success_rate+(1-t)*min_success_rate) {
    //         ss = s+1;
    //         r = forward_reward;
    //     } else {
    //         ss = s;
    //         r = fail_reward;
    //     }
    // } else if(a==FAST_FORWARD) {
    //     DEBUG_OUT(1,"Rate = " << t*max_success_rate_fast+(1-t)*min_success_rate_fast);
    //     if(drand48()<t*max_success_rate_fast+(1-t)*min_success_rate_fast) {
    //         ss = s+1;
    //         r = fast_forward_reward;
    //     } else {
    //         ss = s;
    //         r = fail_reward;
    //     }
    // }
    // // clamp to allowed states and return
    // ss = util::clamp<int>(0,states.size()-1,ss);
    return state_reward_pair_t(ss,r);
}

QString DynamicTightRope::action_name(const action_t & a) const {
    return action_names[a];
}

QString DynamicTightRope::state_name(const state_t & s) const {
    return state_names[s];
}
