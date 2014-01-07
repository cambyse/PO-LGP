#include "Environment.h"

void Environment::get_spaces(action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r) const {
    a = action_space;
    o = observation_space;
    r = reward_space;
}
