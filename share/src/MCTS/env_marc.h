#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <tuple>

#include "../../include/MCTS_Environment/AbstractEnvironment.h"

/** This is an abstraction of an environment for MCTS. The environment essentially only needs to simulate (=transition
 *  forward the state for given actions), and for each transition return the observation and reward. Additionally, it
 *  needs to provide the set of feasible decisions for the current state. For the MCTS solver, states, actions, and
 *  rewards are fully abstract entities -- they can only be referred to via 'handles'. */
struct MCTS_Environment {
    /** A generic State-Action-Observation object as abstraction of a real state, action, or observation. The environment can,
     *  via dynamic casting, get the semantics back. The MCTS solver should not use any other properties than equality. */
    struct SAO {
      virtual ~SAO(){}
      virtual bool operator==(const SAO & other) const { return true; } ///< why true? Because a NIL object (e.g. non-observation) is always equal to NIL
      virtual bool operator!=(const SAO & other) const { return !(*this==other); }
      virtual void write(std::ostream& os) const { os <<"NIL_SAO"; }
        virtual size_t get_hash() const {
            std::cout << "NOT IMPLEMENTED YET -- HARD EXIT!!!" << std::endl;
            exit(-1);
        }
    };
    typedef std::shared_ptr<const SAO> Handle;

    MCTS_Environment() = default;
  virtual ~MCTS_Environment(){}

    /// Perform the action; return the resulting observation and reward
    virtual std::pair<Handle, double> transition(const Handle& action) = 0;

    /// Perform a random action
    virtual std::pair<Handle, double> transition_randomly(){
      std::vector<Handle> actions = get_actions();
      return transition(actions[rand()%actions.size()]);
    }

    /// Get the available actions in the current state
    virtual const std::vector<Handle> get_actions() = 0;

    /// Get the current state
    virtual const Handle get_state() = 0;

    /// Return whether the current state is a terminal state
    virtual bool is_terminal_state() const = 0;

    /// Makes the current state the future start state set by reset_state().
    virtual void make_current_state_default() = 0;

    /// Reset the environment's state to the start state
    virtual void reset_state() = 0;

    enum InfoTag{ hasTerminal, isDeterministic, hasMaxReward, getMaxReward, hasMinReward, getMinReward, isMarkov };
    virtual bool get_info(InfoTag tag) const = 0;
    virtual double get_info_value(InfoTag tag) const = 0;
};

class InterfaceMarc: public AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct InterfaceMarcAction: public Action {
        InterfaceMarcAction(MCTS_Environment::Handle action): action(action) {}
        virtual bool operator==(const Action & other) const {
            auto interface_action = dynamic_cast<const InterfaceMarcAction *>(&other);
            return interface_action!=nullptr && *(interface_action->action)==*action;
        }
        virtual size_t get_hash() const {
            return action->get_hash();
        }
        virtual void write(std::ostream & out) const {
            action->write(out);
        }
        MCTS_Environment::Handle action;
    };
    struct InterfaceMarcObservation: public Observation {
        InterfaceMarcObservation(MCTS_Environment::Handle observation): observation(observation) {}
        virtual bool operator==(const Observation & other) const {
            auto interface_observation = dynamic_cast<const InterfaceMarcObservation *>(&other);
            return interface_observation!=nullptr && *(interface_observation->observation)==*(observation);
        }
        virtual size_t get_hash() const {
            return observation->get_hash();
        }
        virtual void write(std::ostream & out) const {
            observation->write(out);
        }
        MCTS_Environment::Handle observation;
    };

    //----members----//
    std::shared_ptr<MCTS_Environment> env_marc;

    //----methods----//
public:
    InterfaceMarc(std::shared_ptr<MCTS_Environment> env_marc): env_marc(env_marc) {}
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) {
        auto interface_action = std::dynamic_pointer_cast<const InterfaceMarcAction>(action_handle);
        assert(interface_action!=nullptr);
        auto return_value = env_marc->transition(interface_action->action);
        return observation_reward_pair_t(observation_handle_t(new InterfaceMarcObservation(return_value.first)),return_value.second);
    }
    template<class C>
        static std::shared_ptr<AbstractEnvironment> makeAbstractEnvironment(C * env) {
        auto mcts = dynamic_cast<MCTS_Environment*>(env);
        assert(mcts!=nullptr);
        return std::shared_ptr<AbstractEnvironment>(
            std::make_shared<InterfaceMarc>(
                std::shared_ptr<MCTS_Environment>(mcts)));
    }
    virtual action_container_t get_actions() {
        action_container_t action_container;
        for(auto action : env_marc->get_actions()) {
            action_container.push_back(action_handle_t(new InterfaceMarcAction(action)));
        }
        return action_container;
    }
    virtual void make_current_state_default() {
        env_marc->make_current_state_default();
    }
    virtual void reset_state() {
        env_marc->reset_state();
    }
    virtual bool has_terminal_state() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasTerminal);
    }
    virtual bool is_terminal_state() const {
        return env_marc->is_terminal_state();
    }
    virtual bool is_deterministic() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::isDeterministic);
    }
    virtual bool has_max_reward() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasMaxReward);
    }
    virtual reward_t max_reward() const {
        return (reward_t)env_marc->get_info_value(MCTS_Environment::InfoTag::getMaxReward);
    }
    virtual bool has_min_reward() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasMinReward);
    }
    virtual reward_t min_reward() const {
        return (reward_t)env_marc->get_info_value(MCTS_Environment::InfoTag::getMinReward);
    }
    virtual bool is_markov() const {
        return env_marc->get_info(MCTS_Environment::InfoTag::isMarkov);
    }
};
