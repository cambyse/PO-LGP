/*
 * KMDPState.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: robert
 */

#include "KMDPState.h"

#include "debug.h"

KMDPState::KMDPState(): k_mdp_state_deque(Data::k) {}

KMDPState::~KMDPState() {}

void KMDPState::new_state(const Data::data_point_t& data_point) {
    k_mdp_state_deque.pop_back();
    k_mdp_state_deque.push_front(data_point);
}

void KMDPState::new_state(const Data::action_t& action, const Data::state_t& state, const Data::reward_t& reward) {
    new_state(Data::data_point_t(action,state,reward));
}

Data::k_mdp_state_t KMDPState::get_k_mdp_state() {
    Data::k_mdp_state_t k_mdp_state;
    for(std::deque<Data::data_point_t>::const_iterator it=k_mdp_state_deque.begin(); it!=k_mdp_state_deque.end(); ++it) {
        k_mdp_state.push_back(*it);
    }
    if(k_mdp_state.size()!=Data::k) {
        DEBUG_OUT(0,"Error: something is wrong with the k-MDP state size.");
    }
    return k_mdp_state;
}
