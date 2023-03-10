#include "HistoryObserver.h"

#include <representation/DoublyLinkedInstance.h>

#define DEBUG_STRING "HistoryObserver: "
#define DEBUG_LEVEL 1
#include "util/debug.h"

HistoryObserver::HistoryObserver():
    number_of_data_points(0)
{}

void HistoryObserver::add_action_observation_reward_tripel(
        const action_ptr_t& action,
        const observation_ptr_t& observation,
        const reward_ptr_t& reward,
        const bool& new_episode
    ) {

    if(instance_data.size()==0 || new_episode) {
        // start new episode
        instance_ptr_t new_instance = DoublyLinkedInstance::create(action,observation,reward);
        instance_data.push_back(new_instance);
    } else {
        // add to existing episode
        instance_ptr_t current_instance = instance_data.back();
        instance_ptr_t new_instance = current_instance->append(action,observation,reward);
        instance_data.back() = new_instance;
    }

    DEBUG_OUT(2, "added (action,observation,reward) = (" <<
              action << "," <<
              observation << "," <<
              reward << ") to " << (new_episode?"new":"old") << " epsisode" );

    // increment number of data points
    ++number_of_data_points;
}

void HistoryObserver::clear_data() {
    for(auto ins : instance_data) {
        ins->detach_all();
    }
    instance_data.clear();
    number_of_data_points = 0;
}
