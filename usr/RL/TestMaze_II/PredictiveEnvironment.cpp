#include "PredictiveEnvironment.h"

#include "debug.h"

PredictiveEnvironment::PredictiveEnvironment(): current_instance(nullptr) {}

void PredictiveEnvironment::perform_transition(const action_ptr_t & action) {
    // perform transition
    probability_t prob_threshold = drand48();
    DEBUG_OUT(2,"Prob threshold = " << prob_threshold);
    probability_t prob_accum = 0;
    bool was_set = false;
    for(observation_ptr_t observation_to : observation_space) {
        if(was_set) {
            break;
        }
        for(reward_ptr_t reward : reward_space) {
            if(was_set) {
                break;
            }
            probability_t prob = get_prediction(current_instance, action, observation_to, reward);
            DEBUG_OUT(2,"observation(" << observation_to << "), reward(" << reward << ") --> prob=" << prob);
            prob_accum += prob;
            if(prob_accum>prob_threshold) {
                current_instance = current_instance->append_instance(action, observation_to, reward);
                was_set = true;
                DEBUG_OUT(2,"CHOOSE");
            }
        }
    }
    if(!was_set) {
        DEBUG_OUT(0, "Error: Unnormalized probabilities [sum(p)=" << prob_accum << "]--> no transition performed." );
    }
}

void PredictiveEnvironment::perform_transition(const action_ptr_t& a, observation_ptr_t& final_observation, reward_ptr_t& r) {
    perform_transition(a);
    final_observation = current_instance->observation;
    r = current_instance->reward;
}

void PredictiveEnvironment::get_spaces(action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r) const {
    a = action_space;
    o = observation_space;
    r = reward_space;
}
