#ifndef BOTTLENECKHALLWAY_H_
#define BOTTLENECKHALLWAY_H_

#include "Environment.h"

class BottleNeckHallway: public Environment {
    //----typedefs/classes----//

    //----members----//
private:
    int length, action_n;
    double min_prob, max_prob;

    //----methods----//
public:
    BottleNeckHallway(int length, int action_n, double min_prob, double max_prob);
    virtual ~BottleNeckHallway() = default;
    virtual state_reward_pair_t sample(const state_t &, const action_t &) const override;
    virtual bool has_terminal_state() const override { return true; }
    virtual bool is_terminal_state(state_t s) const override { return s>=length-1; }
};

#endif /* BOTTLENECKHALLWAY_H_ */
