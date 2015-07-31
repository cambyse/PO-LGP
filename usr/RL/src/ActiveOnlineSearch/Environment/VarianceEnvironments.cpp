#include "VarianceEnvironments.h"

#include <util/debug.h>

typedef DelayedLowVarianceSubOptimalReward DLVSOR;
typedef LowVarianceSubOptimalReward LVSOR;

LVSOR::LowVarianceSubOptimalReward(double sub_optimal_return_):
    sub_optimal_return(sub_optimal_return_) {}

LVSOR::observation_reward_pair_t LVSOR::transition(const action_handle_t & action_handle) {
    auto action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
    double reward = 0;
    DEBUG_EXPECT(state==0);
    DEBUG_EXPECT(action!=nullptr);
    DEBUG_EXPECT(action->action==1 || action->action==2);
    state = action->action;
    if(state==1) {
        // low-variance sub-optimal return
        reward = sub_optimal_return;
    } else {
        // high-variance optimal return
        if(rand()%2==0) {
            reward = 1;
        } else {
            reward = 0;
        }
    }
    return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)),reward);
}

LVSOR::action_container_t LVSOR::get_actions() {
    return action_container_t({action_handle_t(new IntegerAction(1)),
                action_handle_t(new IntegerAction(2))});
}

DLVSOR::DelayedLowVarianceSubOptimalReward(int action_n_,
                                           int depth_,
                                           double sub_optimal_return_):
    action_n(action_n_), depth(depth_), sub_optimal_return(sub_optimal_return_) {}

DLVSOR::observation_reward_pair_t DLVSOR::transition(const action_handle_t & action_handle) {
    auto action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
    double reward = 0;
    DEBUG_EXPECT(abs(state)<depth);
    DEBUG_EXPECT(action!=nullptr);
    //---------------------------------------------------------------------- The
    // absolute value of the state corresponds to the time; the sign corresponds
    // to whether we are in high-variance or low-variance paths
    // ----------------------------------------------------------------------
    // advance in time
    if(state>=0) ++state;
    else --state;
    // switch to "low-variance" if action is not 0
    if(action->action!=0 && state>0) state *= -1;
    // give certain sub-optimal reward for low-variance and reward of 1 50% of
    // the time otherwise
    if(abs(state)==depth && (state>0 || rand()%2==0)) reward = 1;
    else reward = sub_optimal_return;
    return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)),reward);
}

DLVSOR::action_container_t DLVSOR::get_actions() {
    action_container_t ret;
    for(int a=0; a<action_n; ++a) {
        ret.push_back(action_handle_t(new IntegerAction(a)));
    }
    return ret;
}
