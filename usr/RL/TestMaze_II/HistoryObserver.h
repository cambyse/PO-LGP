#ifndef HISTORYOBSERVER_H_
#define HISTORYOBSERVER_H_

#include "Config.h"

#include "vector"

class HistoryObserver
{
public:
    USE_CONFIG_TYPEDEFS;
    HistoryObserver();
    virtual ~HistoryObserver() {
        clear_data();
    }
    virtual void add_action_observation_reward_tripel(
            const action_t& action,
            const observation_t& observation,
            const reward_t& reward,
            const bool& new_episode
    );
    virtual void clear_data();
protected:
    std::vector<instance_t*> instance_data;
    size_t number_of_data_points;
};

#endif /* HISTORYOBSERVER_H_ */
