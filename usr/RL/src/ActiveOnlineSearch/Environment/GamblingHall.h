#ifndef GAMBLINGHALL_H_
#define GAMBLINGHALL_H_

#include "Environment.h"

class GamblingHall: public Environment {
    //----typedefs/classes----//

    //----members----//
private:
    int machine_n, time_n;
    double tolerance;

    //----methods----//
public:
    GamblingHall(int machine_n, double tolerance = 0.1);
    virtual ~GamblingHall() = default;
    virtual state_reward_pair_t sample(const state_t &, const action_t &) const override;
    virtual bool has_terminal_state() const override;
    virtual bool is_terminal_state(state_t s) const override;
    virtual QString state_name(const state_t &) const override;
};

#endif /* GAMBLINGHALL_H_ */
