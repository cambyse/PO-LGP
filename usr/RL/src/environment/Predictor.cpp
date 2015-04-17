#include "Predictor.h"

Predictor::probability_map_t Predictor::get_prediction_map(const_instance_ptr_t ins,
                                                           const action_ptr_t& action) const {
    probability_map_t return_map;
    for(observation_ptr_t observation : observation_space) {
        for(reward_ptr_t reward : reward_space) {
            return_map[std::make_tuple(observation,reward)] = get_prediction(ins,action,observation,reward);
        }
    }
    return return_map;
}
