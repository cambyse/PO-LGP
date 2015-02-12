#ifndef DYNAMICTIGHTROPE_H_
#define DYNAMICTIGHTROPE_H_

#include "Environment.h"

class DynamicTightRope: public Environment {
    //----typedefs/classes----//

    //----members----//
public:
    const int position_n;
    static const int velocity_n = 5;
private:
    std::vector<QString> action_names;
    std::vector<QString> state_names;
    enum ACTIONS {ACCELERATE, KEEP_VELOCITY, DECELERATE};

    //----methods----//
public:
    virtual state_reward_pair_t sample(const state_t &, const action_t &) const override;
    virtual QString action_name(const action_t &) const override;
    virtual QString state_name(const state_t &) const override;
    bool has_terminal_state() const override {return true;}
    bool is_terminal_state(state_t s) const override {return s==states.size()-1;}
    DynamicTightRope(int n = 15);
    virtual ~DynamicTightRope() = default;
private:
    double success_probability(const int & pos, const int & vel) const;
};

#endif /* DYNAMICTIGHTROPE_H_ */
