#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <QString>

#include <tuple>
#include <vector>

class Environment {

    //----typedefs/classes----//
public:
    typedef int state_t;
    typedef int action_t;
    typedef double reward_t;
    typedef std::tuple<state_t,reward_t> state_reward_pair_t;

    //----members----//
    std::vector<action_t> actions;
    std::vector<state_t> states;

    //----methods----//
public:
    virtual state_reward_pair_t sample(const state_t &, const action_t &) const = 0;
    virtual QString action_name(const action_t & a) const {return QString::number(a);}
    virtual QString state_name(const state_t & s) const {return QString::number(s);}
    virtual bool has_terminal_state() const = 0;
    virtual bool is_terminal_state(state_t) const = 0;
    virtual state_t default_state() const {return 0;}

public:
    Environment(const std::vector<action_t> & a = std::vector<action_t>(),
                const std::vector<state_t> & s = std::vector<state_t>()):
    actions(a), states(s) {}
    virtual ~Environment() = default;
};

#endif /* ENVIRONMENT_H_ */
