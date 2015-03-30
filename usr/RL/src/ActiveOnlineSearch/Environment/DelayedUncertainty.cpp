#include "DelayedUncertainty.h"

#include <util/util.h>
#include <util/pretty_printer.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using namespace ND_vector;
using util::Range;
using util::convert_1D_to_ND_index;
using util::convert_ND_to_1D_index;

DelayedUncertainty::DelayedUncertainty(int options = 2,
                                       int time_horizon = 1,
                                       ND_vector::vec_double_2D _probabilities):
    Environment(util::range_vector(options),util::range_vector(options*time_horizon)),
    branch_n(options),
    action_n(options),
    time_horizon(time_horizon),
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
    DEBUG_OUT(1,"Actions " << actions);
        DEBUG_OUT(1,"States");
    for(auto s : states) {
        DEBUG_OUT(1,"    " << s << ": " << state_name(s));
    }
}

DelayedUncertainty::state_reward_pair_t DelayedUncertainty::sample(const state_t & state,
                                                                   const action_t & action) const {
    auto branch_time = convert_1D_to_ND_index(state,{branch_n,time_horizon});
    branch_time[1] += 1;
    reward_t reward;
    DEBUG_OUT(2, "state: " << state);
    DEBUG_OUT(2, "branch_time: " << branch_time);
    DEBUG_OUT(2, "branch: " << branch_time[0] << ", Action " << action);
    DEBUG_OUT(2, "probabilities: " << probabilities);
    if(branch_time[0]==0) {
        branch_time[0] = action;
        reward = 0;
    } else if(branch_time[1]==time_horizon) {
        double prob = probabilities[branch_time[0]][action];
        if(drand48()<prob) {
            reward = 1;
        } else {
            reward = 0;
        }
    }
    return state_reward_pair_t(convert_ND_to_1D_index(branch_time,{branch_n,time_horizon}),reward);
}

bool DelayedUncertainty::is_terminal_state(state_t state) const {
    return convert_1D_to_ND_index(state,{branch_n,time_horizon})[1]>=(time_horizon-1);
}

QString DelayedUncertainty::state_name(const state_t & state) const {
    auto branch_time = convert_1D_to_ND_index(state,{branch_n,time_horizon});
    return QString("b=%1, t=%2").arg(branch_time[0]).arg(branch_time[1]);
}
