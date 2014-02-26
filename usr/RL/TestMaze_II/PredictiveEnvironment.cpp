#include "PredictiveEnvironment.h"

#define DEBUG_LEVEL 0
#include "debug.h"

PredictiveEnvironment::PredictiveEnvironment(action_ptr_t as, observation_ptr_t os, reward_ptr_t rs):
    current_instance(nullptr)
{
    set_spaces(as, os, rs);
}

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
        DEBUG_ERROR("Unnormalized probabilities [sum(p)=" << prob_accum << "]--> no transition performed." );
    }
}

void PredictiveEnvironment::perform_transition(const action_ptr_t& a, observation_ptr_t& final_observation, reward_ptr_t& r) {
    perform_transition(a);
    final_observation = current_instance->observation;
    r = current_instance->reward;
}
