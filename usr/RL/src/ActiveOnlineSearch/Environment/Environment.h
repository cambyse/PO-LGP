#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <QString>
#include <util/QtUtil.h>

#include <tuple>
#include <vector>

#include <MCTS_Environment/AbstractFiniteEnvironment.h>

class Environment: public AbstractFiniteEnvironment<int,int> {

    //----typedefs/classes----//

    //----members----//

    //----methods----//
public:
    Environment(const std::vector<action_t> & actions,
                const std::vector<state_t> & states):
        AbstractFiniteEnvironment<int,int>(actions,states) {}
    Environment(const std::vector<int> & actions,
                const std::vector<int> & states):
        AbstractFiniteEnvironment<int,int>(actions,states) {}
    virtual ~Environment() = default;
    virtual state_reward_pair_t transition(const state_t &, const action_t &) const = 0;
    virtual QString action_name(const action_t & a) const {return QString::number(a);}
    virtual QString state_name(const state_t & s) const {return QString::number(s);}
    virtual bool is_terminal_state() const override final {return is_terminal_state(state);}
    virtual bool is_terminal_state(state_t s) const = 0;
};

#endif /* ENVIRONMENT_H_ */
