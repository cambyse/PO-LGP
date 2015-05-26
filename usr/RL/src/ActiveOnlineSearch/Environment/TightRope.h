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
    virtual state_reward_pair_t transition(const state_t &, const action_t &) const override;
    virtual QString action_name(const action_t &) const override;
    virtual QString state_name(const state_t &) const override;
    bool has_terminal_state() const override {return true;}
    bool is_terminal_state(state_t s) const override {return s==(int)state_list.size()-1;}
public:
    TightRope(int n = 15);
    virtual ~TightRope() = default;
};

#endif /* TIGHTROPE_H_ */
