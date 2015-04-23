#ifndef DELAYEDUNCERTAINTY_H_
#define DELAYEDUNCERTAINTY_H_

#include "Environment.h"

#include <util/ND_vector.h>

class DelayedUncertainty: public Environment {
    //----typedefs/classes----//

    //----members----//
private:
    int branch_n, action_n, time_steps_n;
    ND_vector::vec_double_2D probabilities;

    //----methods----//
public:
    DelayedUncertainty(int options,
                       int time_steps_n ,
                       ND_vector::vec_double_2D probabilities = {});
    virtual ~DelayedUncertainty() = default;
    virtual state_reward_pair_t sample(const state_t &, const action_t &) const override;
    virtual bool has_terminal_state() const override { return true; }
    virtual bool is_terminal_state(state_t s) const override;
    virtual QString state_name(const state_t &) const override;
};

#endif /* DELAYEDUNCERTAINTY_H_ */
