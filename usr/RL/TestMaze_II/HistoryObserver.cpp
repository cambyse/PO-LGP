
#include "HistoryObserver.h"
#define DEBUG_STRING "HistoryObserver: "
#define DEBUG_LEVEL 1
#include "debug.h"

HistoryObserver::HistoryObserver(): instance_data(nullptr) {}

HistoryObserver::~HistoryObserver() { delete instance_data; }

void HistoryObserver::add_action_state_reward_tripel(
        const action_t& action,
        const state_t& state,
        const reward_t& reward
    ) {
    if(instance_data==nullptr) {
        instance_data = instance_t::create(action,state,reward);
        instance_data->set_container();
    } else {
        instance_data = instance_data->append_instance(action,state,reward);
    }
    DEBUG_OUT(2, "added (action,state,reward) = (" << action << "," << state << "," << reward << ")" );
}
