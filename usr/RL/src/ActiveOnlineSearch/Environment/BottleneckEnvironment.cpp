#include "BottleneckEnvironment.h"

#include <util/debug.h>

BottleneckEnvironment::BottleneckEnvironment(int branching, int branch_length):
    branching(branching), branch_length(branch_length)
{
    DEBUG_EXPECT(0,branch_length>1);
}

BottleneckEnvironment::observation_reward_pair_t BottleneckEnvironment::transition(const action_handle_t & action_handle) {
    auto x = std::dynamic_pointer_cast<const BottleneckAction>(action_handle);
    DEBUG_EXPECT(0,x!=nullptr);
    reward_t reward = 0;
    double prob = (double)(state.branch-1)/(branching-1);
    if(state.step==0) {
        state.branch = x->action;
        state.step = 1;
    } else if(state.step==branch_length-1) {
        ++state.step;
        if(drand48()<prob) {
            reward = 0.1*(1-prob);
        }
    } else {
        ++state.step;
    }
    return observation_reward_pair_t(observation_handle_t(new BottleneckObservation(state)), reward);
}

BottleneckEnvironment::action_container_t BottleneckEnvironment::get_actions() {
    if(state.step==0) {
        action_container_t actions;
        for(int a=1; a<=branching; ++a) {
            actions.push_back(action_handle_t(new BottleneckAction(a)));
        }
        return actions;
    } else {
        return action_container_t({action_handle_t(new BottleneckAction(0))});
    }
}
