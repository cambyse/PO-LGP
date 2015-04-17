#include "SpaceManager.h"

void SpaceManager::get_spaces(action_ptr_t & a,
                              observation_ptr_t & o,
                              reward_ptr_t & r) const {
    a = action_space;
    o = observation_space;
    r = reward_space;
}

void SpaceManager::set_spaces(const action_ptr_t & a,
                              const observation_ptr_t & o,
                              const reward_ptr_t & r) {
    action_space = a;
    observation_space = o;
    reward_space = r;
}

void SpaceManager::adopt_spaces(const SpaceManager & s) {
    set_spaces(s.action_space, s.observation_space, s.reward_space);
}
