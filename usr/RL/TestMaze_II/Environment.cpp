#include "Environment.h"

Environment::Environment(action_ptr_t as, observation_ptr_t os, reward_ptr_t rs):
    action_space(as), observation_space(os), reward_space(rs)
{}

void Environment::get_spaces(action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r) const {
    a = action_space;
    o = observation_space;
    r = reward_space;
}
