#include "DynamicTightRope.h"

#include <util/util.h>
#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using util::Range;
using util::get_ND_index;
using util::convert_ND_to_1D_index;
using util::convert_1D_to_ND_index;
using util::clamp;
using std::tuple;
using std::vector;

DynamicTightRope::DynamicTightRope(int pos, int vel):
    Environment(util::range_vector(3), util::range_vector(vel*pos)),
    position_n(pos),
    velocity_n(vel),
    action_names({"accel", "keep", "decel"})
{
    for(state_t state : states) {
        auto pos_vel = convert_1D_to_ND_index(state,{pos,vel});
        state_names[state] = QString("pos %1, vel %2").arg(pos_vel[0]).arg(pos_vel[1]);
    }
}

DynamicTightRope::state_reward_pair_t DynamicTightRope::sample(const state_t & s,
                                                               const action_t & a) const {
    // position/velocity
    RETURN_TUPLE(int,pos,int,vel) = get_ND_index<2>::from(s,{position_n,velocity_n});
    DEBUG_OUT(1,"pos = " << pos << ", vel = " << vel);

    // reward
    reward_t r = 0;

    // adapt velocity
    if(a==ACCELERATE) {
        vel += 1;
    } else if(a==DECELERATE) {
        vel -= 1;
    }
    vel = clamp(0,velocity_n-1,vel);

    // success probability
    double prob = success_probability(pos,vel);

    // perform action
    if(drand48()<prob) {
        //---------//
        // success //
        //---------//

        // change position
        //pos += vel;
        pos += 1;
        pos = clamp(0,position_n-1,pos);

        // give reward for moving forward
        r = vel;
    } else {
        //---------//
        // failure //
        //---------//

        // you hurt yourself (the more the faster you go)
        r = -vel*vel;

        // falling brings velocity to zero (position stays the same)
        //vel = 0;
    }

    return state_reward_pair_t(convert_ND_to_1D_index({pos,vel},{position_n,velocity_n}),r);
}

QString DynamicTightRope::action_name(const action_t & a) const {
    return action_names[a];
}

QString DynamicTightRope::state_name(const state_t & s) const {
    return state_names[s];
}

std::tuple<int,int> DynamicTightRope::get_position_and_velocity(const state_t & state) const {
    return get_ND_index<2>::from(state,{position_n,velocity_n});
}

double DynamicTightRope::success_probability(const int & pos, const int & vel) const {

    // success probabilities
    double max_success_rate = 0.95;
    double min_success_rate = 0.05;

    // relative position on the rope -- in [0,1] linearly increasing
    double t = ((double)pos)/(position_n-1);

    // "risk" on the rope (bell-shaped) -- in [0,1] with maximum at 0.5
    double risk = exp(-pow(2*(t-0.5),2)/0.15);

    // for low velocity it's always safe but risk increases for higher
    // velocities
    return max_success_rate - (1-min_success_rate)*risk*vel/(velocity_n-1);
}
