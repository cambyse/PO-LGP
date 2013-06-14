#ifndef HISTORYOBSERVER_H_
#define HISTORYOBSERVER_H_

#include "Config.h"

class HistoryObserver
{
public:
    USE_REPRESENTATION_TYPEDEFS;
    HistoryObserver();
    virtual ~HistoryObserver();
    virtual void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );
    virtual void clear_data() {
        delete instance_data;
        instance_data = nullptr;
    }
protected:
    instance_t * instance_data;
};

#endif /* HISTORYOBSERVER_H_ */
