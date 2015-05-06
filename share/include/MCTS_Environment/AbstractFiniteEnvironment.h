#ifndef ABSTRACTFINITEENVIRONMENT_H_
#define ABSTRACTFINITEENVIRONMENT_H_

#include "AbstractEnvironment.h"

template<class ACTION = int, class STATE = int>
class AbstractFiniteEnvironment: public AbstractEnvironment {

    //----typedefs/classes----//

public:

    template <class C, class T>
    class TypeWrapper {
        // types
    public:
        typedef T value_t;
        struct hash {
            size_t operator()(const TypeWrapper & x) const {return std::hash<int>((int)x);}
        };
        // methods
    public:
        TypeWrapper(const T& val): value(val) {}
        operator T() const {return value;}
        C& operator=(const T &rhs) {this->value=rhs; return *this;}
        bool operator==(const T &other) const {return this->value==other;}
        bool operator!=(const T &other) const {return this->value!=other;}
        bool operator<(const T &other) const {return this->value<other;}
        bool operator>(const T &other) const {return this->value>other;}
        bool operator<=(const T &other) const {return this->value<=other;}
        bool operator>=(const T &other) const {return this->value>=other;}
        T & operator+=(const T &rhs) {return this->value+=rhs;}
        T & operator-=(const T &rhs) {return this->value-=rhs;}
    protected:
        T value;
    };

    class FiniteAction: public AbstractEnvironment::Action,
                       public TypeWrapper<FiniteAction,ACTION> {
    public:
        FiniteAction(ACTION action):
            TypeWrapper<FiniteAction,ACTION>(action) {}
        virtual ~FiniteAction() = default;
        virtual bool operator==(const Action & other) const {
            auto finite = dynamic_cast<const FiniteAction *>(&other);
            return finite!=nullptr && finite->value==this->value;
        }
    };
    typedef FiniteAction action_t;

    class FiniteState: public AbstractEnvironment::State,
                       public TypeWrapper<FiniteState,STATE> {
    public:
        FiniteState(STATE state):
            TypeWrapper<FiniteState,STATE>(state) {}
        virtual ~FiniteState() = default;
        virtual bool operator==(const State & other) const {
            auto finite = dynamic_cast<const FiniteState *>(&other);
            return finite!=nullptr && finite->value==this->value;
        }
    };
    typedef FiniteState state_t;

    class FiniteObservation: public AbstractEnvironment::Observation,
                       public TypeWrapper<FiniteObservation,STATE> {
    public:
        FiniteObservation(STATE observation):
            TypeWrapper<FiniteObservation,STATE>(observation) {}
        virtual ~FiniteObservation() = default;
        virtual bool operator==(const Observation & other) const {
            auto finite = dynamic_cast<const FiniteObservation *>(&other);
            return finite!=nullptr && finite->value==this->value;
        }
    };
    typedef FiniteObservation observation_t;

    typedef std::pair<state_t,reward_t> state_reward_pair_t;

    //----members----//

public:

    const std::vector<action_t> action_list;
    const action_container_t action_handle_list;
    const std::vector<state_t> state_list;
    state_t state;

    //----methods----//

public:

    AbstractFiniteEnvironment(const std::vector<action_t> action_list,
                              const std::vector<state_t> state_list):
    action_list(action_list),
    action_handle_list(construct_action_container(action_list)),
    state_list(state_list),
    state(state_list[0])
    {}

    AbstractFiniteEnvironment(const std::vector<ACTION> action_list,
                              const std::vector<STATE> state_list):
        AbstractFiniteEnvironment(convert_vector<ACTION,action_t>(action_list),
                                  convert_vector<STATE,state_t>(state_list)) {}

    virtual ~AbstractFiniteEnvironment() = default;

    template<class FROM, class TO>
    static std::vector<TO> convert_vector(const std::vector<FROM> & vec) {
        std::vector<TO> ret;
        for(auto elem : vec) {
            ret.push_back((TO)elem);
        }
        return ret;
    }

    static action_container_t construct_action_container(const std::vector<action_t> & action_list) {
        action_container_t action_handle_list;
        for(action_t action : action_list) {
            action_handle_list.push_back(make_action_handle(action));
        }
        return action_handle_list;
    }

    virtual state_reward_pair_t transition(const state_t state, const action_t action) = 0;

    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) final {
        auto action = std::dynamic_pointer_cast<const action_t>(action_handle);
        assert(action!=nullptr);
        auto state_reward = transition(state,*action);
        state = state_reward.first;
        return observation_reward_pair_t(make_observation_handle(state_reward.first),state_reward.second);
    }

    virtual const action_container_t get_actions() final {return action_handle_list;}

    virtual const state_handle_t get_state_handle() final {return make_state_handle(state);}

    virtual void set_state(const state_handle_t & state_handle) final {
        auto finite_state = std::dynamic_pointer_cast<const state_t>(state_handle);
        assert(finite_state!=nullptr);
        state=*finite_state;
    }

    virtual bool is_markov() const final {return true;}
};

#endif /* ABSTRACTFINITEENVIRONMENT_H_ */
