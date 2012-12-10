/*
 * QIteration.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: robert
 */

#include "QIteration.h"

#include "float.h"

#define DEBUG_LEVEL 2
#include "debug.h"

typedef Data::k_mdp_state_idx_t k_mdp_state_idx_t;
typedef Data::action_idx_t action_idx_t;
typedef Data::state_idx_t state_idx_t;

QIteration::QIteration(const double& d):
        discount(d),
        prediction(Data::k_mdp_state_n*Data::action_n*Data::state_n*Data::reward_n,0),
        state_value(Data::k_mdp_state_n,0),
        state_action_value(Data::k_mdp_state_n*Data::action_n,0)
{}

QIteration::~QIteration() {}

void QIteration::set_prediction(k_mdp_state_t state_from, action_t action, state_t state_to, reward_t reward, probability_t prob) {
    prediction[get_prediction_idx(state_from,action,state_to,reward)] = prob;
}

void QIteration::set_discount(const double& d) {
    discount = d;
}

double QIteration::get_discount() {
    return discount;
}

QIteration::value_t QIteration::iterate() {

    //-------------------------//
    //    Update Q-Function    //
    //-------------------------//

    // for all state action pairs
    for(Data::k_mdp_state_idx_t k_mdp_state_from_idx=0;
            k_mdp_state_from_idx<Data::k_mdp_state_n;
            ++k_mdp_state_from_idx) {
        k_mdp_state_t k_mdp_state_from = Data::k_mdp_state_from_idx(k_mdp_state_from_idx);
        for(action_t action=0; action<Data::action_n; ++action) {
            idx_t state_action_idx = get_state_action_idx(k_mdp_state_from,action);
            state_action_value[state_action_idx] = 0;

            // for all outputs
            for(Data::OutputIterator out_it; !out_it.end(); ++out_it){
                k_mdp_state_t k_mdp_state_to(1,Data::data_point_t(action,(*out_it).state,(*out_it).reward));
                for(unsigned int idx=0; idx<k_mdp_state_from.size()-1; ++idx) {
                    k_mdp_state_to.push_back(k_mdp_state_from[idx]);
                }
                Data::k_mdp_state_idx_t k_mdp_state_to_idx = Data::idx_from_k_mdp_state(k_mdp_state_to);
                idx_t prediction_idx = get_prediction_idx(k_mdp_state_from,action,(*out_it).state,(*out_it).reward);
                state_action_value[state_action_idx] +=
                        prediction[prediction_idx] * (
                                (*out_it).reward + discount * state_value[k_mdp_state_to_idx]
                        );
            }

        }
    }

    //-----------------------------//
    //    Update Value-Function    //
    //-----------------------------//

    value_t max_value_diff = -DBL_MAX;
    value_t max_value = -DBL_MAX;
    for(Data::k_mdp_state_idx_t k_mdp_state_idx=0;
            k_mdp_state_idx<Data::k_mdp_state_n;
            ++k_mdp_state_idx) {
        k_mdp_state_t k_mdp_state = Data::k_mdp_state_from_idx(k_mdp_state_idx);
        value_t old_value = state_value[k_mdp_state_idx];
        state_value[k_mdp_state_idx] = -DBL_MAX;
        for(action_t action=0; action<Data::action_n; ++action) {
            if(state_action_value[get_state_action_idx(k_mdp_state,action)]>state_value[k_mdp_state_idx]) {
                state_value[k_mdp_state_idx] = state_action_value[get_state_action_idx(k_mdp_state,action)];
            }
        }
        if(fabs(old_value-state_value[k_mdp_state_idx])>max_value_diff) {
            max_value_diff = fabs(old_value-state_value[k_mdp_state_idx]);
        }
        if(state_value[k_mdp_state_idx]>max_value) {
            max_value = state_value[k_mdp_state_idx];
        }
    }
    DEBUG_OUT(1, "Max value      = " << max_value);
    DEBUG_OUT(1, "Max value diff = " << max_value_diff);

    return max_value_diff;
}

QIteration::action_t QIteration::optimal_action(const k_mdp_state_t& k_mdp_state) {
    std::vector<action_t> optimal_action;
    value_t max_value = -DBL_MAX;
    for(action_t action=0; action<Data::action_n; ++action) {
        if(state_action_value[get_state_action_idx(k_mdp_state,action)]>max_value) {
            max_value = state_action_value[get_state_action_idx(k_mdp_state,action)];
            optimal_action.assign(1,action);
        } else if(state_action_value[get_state_action_idx(k_mdp_state,action)]==max_value) {
            optimal_action.push_back(action);
        }
    }
    return optimal_action[rand()%optimal_action.size()];
}

QIteration::idx_t QIteration::get_prediction_idx(k_mdp_state_t state_from, action_t action, state_t state_to, reward_t reward) {
    return Data::idx_from_k_mdp_state(state_from) +
            Data::k_mdp_state_n*action +
            Data::k_mdp_state_n*Data::action_n*state_to +
            Data::k_mdp_state_n*Data::action_n*Data::state_n*Data::idx_from_reward(reward);
}

QIteration::idx_t QIteration::get_state_action_idx(k_mdp_state_t state, action_t action) {
    return Data::idx_from_k_mdp_state(state) + Data::k_mdp_state_n*action;
}
