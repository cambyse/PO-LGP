#ifndef TIGHTROPE_H_
#define TIGHTROPE_H_

#include "Environment.h"

class TightRope: public Environment {
    //----typedefs/classes----//

    //----members----//
private:
    std::vector<QString> action_names;
    std::vector<QString> state_names;
    enum ACTIONS {FORWARD, FAST_FORWARD};

    //----methods----//
public:
    virtual state_reward_pair_t sample(const state_t &, const action_t &) const override;
    virtual QString action_name(const action_t &) const override;
    virtual QString state_name(const state_t &) const override;
public:
    TightRope(int n = 10);
    virtual ~TightRope() = default;
};

#endif /* TIGHTROPE_H_ */
