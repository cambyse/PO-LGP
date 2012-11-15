
#include "ValueIteration.h"

#include "debug.h"

double ValueIteration::get_transition_probability(const Data::state_t& state_from, const Data::action_t& action, const Data::state_t& state_to) {
    return probability_table[std::make_tuple(state_from,action,state_to)];
}


void ValueIteration::set_transition_probability(const Data::state_t& state_from, const Data::action_t& action, const Data::state_t& state_to, const double& p) {
    probability_table[std::make_tuple(state_from,action,state_to)] = p;
    state_action_value_table->insert(std::make_pair(std::make_tuple(state_from,action),0));
    old_state_action_value_table->insert(std::make_pair(std::make_tuple(state_from,action),0));
    state_value_table.insert(std::make_pair(state_to,0));
}


double ValueIteration::get_expected_reward(const Data::state_t& state) {
    return reward_table[state];
}


void ValueIteration::set_expected_reward(const Data::state_t& state, const double& r) {
    reward_table[state] = r;
}


void ValueIteration::iterate() {
    // swap tables
    state_action_value_table_t * tmp_state_action_value_table = old_state_action_value_table;
    old_state_action_value_table = state_action_value_table;
    state_action_value_table = tmp_state_action_value_table;

    // clear state action values
    for(typename state_action_value_table_t::iterator it = state_action_value_table->begin(); it != state_action_value_table->end(); ++it) {
        (*it).second = 0;
    }
    // update state action values
    for(typename probability_table_t::iterator it = probability_table.begin(); it != probability_table.end(); ++it) {
        Data::state_t state_from = std::get<0>(it->first);
        Data::action_t action = std::get<1>(it->first);
        Data::state_t state_to = std::get<2>(it->first);
        double prob = it->second;
        typename state_value_table_t::iterator rewIt = reward_table.find(state_to);
        double reward = rewIt==reward_table.end() ? 0 : rewIt->second;

        (*state_action_value_table)[std::make_tuple(state_from,action)] += prob * (reward + discount*state_value_table[state_to]);
    }

    // clear state values
    for(typename state_value_table_t::iterator it = state_value_table.begin(); it != state_value_table.end(); ++it) {
        it->second = -DBL_MAX;
    }
    // update state values
    for(typename state_action_value_table_t::iterator stateActionValueIt = state_action_value_table->begin(); stateActionValueIt != state_action_value_table->end(); ++stateActionValueIt) {
        Data::state_t state= std::get<0>(stateActionValueIt->first);
        Data::action_t action = std::get<1>(stateActionValueIt->first);
        typename state_value_table_t::iterator stateValueIt = state_value_table.find(state);
        if(stateValueIt==state_value_table.end()) {
            DEBUG_OUT(0,"Something is wrong: entry exists in state-action-value-table but not in state-value-table.");
            continue;
        }
        if(stateActionValueIt->second > stateValueIt->second) {
            stateValueIt->second = stateActionValueIt->second;
            state_action_table[state] = action;
        }
    }
}


double ValueIteration::get_value(const Data::state_t& state) {
    return state_value_table[state];
}
