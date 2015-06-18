#ifndef DYNAMICTIGHTROPE_H_
#define DYNAMICTIGHTROPE_H_

#include "Environment.h"

class DynamicTightRope: public Environment {
    //----typedefs/classes----//

    //----members----//
public:
    const int position_n;
    const int velocity_n;
private:
    std::vector<QString> action_names;
    std::vector<QString> state_names;
    enum ACTIONS {ACCELERATE, KEEP_VELOCITY, DECELERATE};

    //----methods----//
public:
    virtual state_reward_pair_t finite_transition(const state_t &, const action_t &) const override;
    virtual QString action_name(const action_t &) const override;
    virtual QString state_name(const state_t &) const override;
    bool has_terminal_state() const override {return true;}
    bool is_terminal_state(state_t s) const override {return s==(int)state_list.size()-1;}
    std::tuple<int,int> get_position_and_velocity(const state_t & state) const;
    DynamicTightRope(int pos, int vel);
    virtual ~DynamicTightRope() = default;
private:
    double success_probability(const int & pos, const int & vel) const;
};

#endif /* DYNAMICTIGHTROPE_H_ */
