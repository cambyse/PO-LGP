#ifndef UNITTESTENVIRONMENT_H_
#define UNITTESTENVIRONMENT_H_

#include "Environment.h"

class UnitTestEnvironment: public Environment {
    //----typedefs/classes----//

    //----members----//

    //----methods----//
public:
    UnitTestEnvironment(): Environment({0,1},{0,1}) {}
    virtual ~UnitTestEnvironment() = default;
    virtual state_reward_pair_t sample(const state_t &, const action_t &) const {
        return state_reward_pair_t(rand()%2,1);
    };
    bool has_terminal_state() const override {return false;}
    bool is_terminal_state(state_t s) const override {return false;}

};

#endif /* UNITTESTENVIRONMENT_H_ */
