#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <QString>
#include <util/QtUtil.h>

#include <tuple>
#include <vector>

#include <MCTS_Environment/AbstractFiniteEnvironment.h>

#include <util/debug.h>

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
    Environment(const std::initializer_list<action_t> action_list,
                const std::initializer_list<state_t> state_list):
        AbstractFiniteEnvironment<int,int>(action_list,state_list) {}
    Environment(const std::initializer_list<int> action_list,
                const std::initializer_list<int> state_list):
        AbstractFiniteEnvironment<int,int>(action_list,state_list) {}
    virtual ~Environment() = default;
    static QString name(const AbstractEnvironment & abstract_environment,
                        const action_handle_t & x) {
        auto environment = dynamic_cast<const Environment *>(&abstract_environment);
        auto action = dynamic_cast<const action_t *>(&(*x));
        if(environment!=nullptr && action!=nullptr) {
            return environment->action_name(*action);
        } else {
            DEBUG_ERROR("Pointer cast failed (" << environment << "/" << action << ")");
            return "a";
        }
    }
    static QString name(const AbstractEnvironment & abstract_environment,
                        const observation_handle_t & x) {
        auto environment = dynamic_cast<const Environment *>(&abstract_environment);
        auto observation = dynamic_cast<const observation_t *>(&(*x));
        if(environment!=nullptr && observation!=nullptr) {
            return environment->observation_name(*observation);
        } else {
            DEBUG_ERROR("Pointer cast failed (" << environment << "/" << observation << ")");
            return "o";
        }
    }
    static QString name(const AbstractEnvironment & abstract_environment,
                        const state_handle_t & x) {
        auto environment = dynamic_cast<const Environment *>(&abstract_environment);
        auto state = dynamic_cast<const state_t *>(&(*x));
        if(environment!=nullptr && state!=nullptr) {
            return environment->state_name(*state);
        } else {
            DEBUG_ERROR("Pointer cast failed (" << environment << "/" << state << ")");
            return "s";
        }
    }
    virtual QString action_name(const action_t & a) const {return QString::number(a);}
    virtual QString state_name(const state_t & s) const {return QString::number(s);}
    virtual QString observation_name(const observation_t & o) const {return state_name((int)o);}
    virtual bool is_terminal_state() const override final {return is_terminal_state(state);}
    virtual bool is_terminal_state(state_t s) const = 0;
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return false;}
    virtual bool has_min_reward() const override {return false;}
    virtual reward_t max_reward() const override {return 0;}
    virtual reward_t min_reward() const override {return 0;}

};

#include <util/debug_exclude.h>

#endif /* ENVIRONMENT_H_ */
