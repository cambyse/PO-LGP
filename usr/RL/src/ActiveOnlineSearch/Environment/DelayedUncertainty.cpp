#include "DelayedUncertainty.h"

#include <util/util.h>
#include <util/pretty_printer.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using namespace ND_vector;
using util::Range;
using util::convert_1D_to_ND_index;
using util::convert_ND_to_1D_index;

DelayedUncertainty::DelayedUncertainty(int options,
                                       int time_steps_n,
                                       ND_vector::vec_double_2D _probabilities):
    Environment(util::range_vector(options),util::range_vector(options*time_steps_n)),
    branch_n(options),
    action_n(options),
    time_steps_n(time_steps_n),
    probabilities(_probabilities)
{
    if(probabilities.size()==0) {
        probabilities.assign(branch_n,vec_double_1D(action_n,0));
        for(int action : Range(action_n)) {
            double extreme_prob = action/(action_n-1);
            for(int branch : Range(branch_n)) {
                double randomness = branch/(branch_n-1);
                probabilities[branch][action] = randomness*0.5 + (1-randomness)*extreme_prob;
            }
        }
    }
    DEBUG_OUT(1,"Probabilities " << probabilities);
    DEBUG_OUT(1,"Actions " << action_list);
        DEBUG_OUT(1,"States");
    for(auto s : state_list) {
        DEBUG_OUT(1,"    " << s << ": " << state_name(s));
    }
}

DelayedUncertainty::state_reward_pair_t DelayedUncertainty::transition(const state_t & state,
                                                                       const action_t & action) const {
    auto branch_time = convert_1D_to_ND_index((int)state,{branch_n,time_steps_n});
    DEBUG_OUT(2, "state: " << state << "(" << state_name(state) << ")");
    DEBUG_OUT(2, "branch_time: " << branch_time);
    DEBUG_OUT(2, "probabilities: " << probabilities);
    reward_t reward = 0;
    if(branch_time[1]==0) {
        // in first transition it's possible to change the branch
        branch_time[0] = action;
    }
    if(branch_time[1]==time_steps_n-2) {
        // in last transition (before reaching a terminal state) you may get a
        // reward
        double prob = probabilities[branch_time[0]][action];
        if(drand48()<prob) {
            reward = 1;
        }
    }
    // advance time
    branch_time[1] += 1;
    // return state and reward
    return state_reward_pair_t(convert_ND_to_1D_index(branch_time,{branch_n,time_steps_n}),reward);
}

bool DelayedUncertainty::is_terminal_state(state_t state) const {
    return convert_1D_to_ND_index((int)state,{branch_n,time_steps_n})[1]>=(time_steps_n-1);
}

QString DelayedUncertainty::state_name(const state_t & state) const {
    auto branch_time = convert_1D_to_ND_index((int)state,{branch_n,time_steps_n});
    return QString("b=%1, t=%2").arg(branch_time[0]).arg(branch_time[1]);
}
