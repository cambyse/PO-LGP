#pragma once

#include <vector>
#include <memory>
#include <tuple>

#include "AbstractEnvironment.h"

/** This is an abstraction of an environment for MCTS. The environment essentially only needs to simulate (=transition
 *  forward the state for given actions), and for each transition return the observation and reward. Additionally, it
 *  needs to provide the set of feasible decisions for the current state. For the MCTS solver, states, actions, and
 *  rewards are fully abstract entities -- they can only be referred to via 'handles'. */
struct MCTS_Environment {
    /** A generic State-Action-Observation object as abstraction of a real state, action, or observation. The environment can,
     *  via dynamic casting, get the semantics back. The MCTS solver should not use any other properties than equality. */
    struct SAO {
      virtual ~SAO() = default;
      virtual bool operator==(const SAO & other) const { return true; } ///< why true? Because a NIL object (e.g. non-observation) is always equal to NIL
      virtual bool operator!=(const SAO & other) const { return !(*this==other); }
      virtual void write(std::ostream& os) const { os <<"NIL_SAO"; }
    };
    typedef std::shared_ptr<const SAO> Handle;
    typedef Handle ActionH;      ///< only to clarify semantics in the declarations below
    typedef Handle ObservationH; ///< only to clarify semantics in the declarations below
    typedef Handle StateH;       ///< only to clarify semantics in the declarations below

    MCTS_Environment() = default;
    virtual ~MCTS_Environment() = default;

    /// Perform the action; return the resulting observation and reward
    virtual std::pair<ObservationH, double> transition(const ActionH& action) = 0;

    /// Perform a random action
    virtual std::pair<ObservationH, double> transition_randomly(){
      std::vector<ActionH> actions = get_actions();
      return transition(actions[rand()%actions.size()]);
    }

    /// Get the available actions in the current state
    virtual const std::vector<ActionH> get_actions() = 0;

    /// Get the current state
    virtual const StateH get_state() = 0;

    /// Return whether the current state is a terminal state
    virtual bool is_terminal_state() const = 0;

    virtual double get_terminal_reward() const { return 0.; }

    /// Set the environment's state to the given state -- DEBATABLE!
    virtual void set_state(const StateH& state) = 0;

    /// Reset the environment's state to the start state
    virtual void reset_state() = 0;

    enum InfoTag{ hasTerminal, isDeterministic, hasMaxReward, getMaxReward, hasMinReward, getMinReward, isMarkov };
    virtual bool get_info(InfoTag tag) const = 0;
    virtual double get_info_value(InfoTag tag) const = 0;
};

#if 0
class InterfaceMarc: public AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct InterfaceMarcAction: public Action {
        InterfaceMarcAction(MCTS_Environment::ActionH action): action(action) {}
        virtual bool operator==(const Action & other) const override {
            auto interface_action = dynamic_cast<const InterfaceMarcAction *>(&other);
            return interface_action!=nullptr && *(interface_action->action)==*action;
        }
        virtual size_t get_hash() const override {
            #warning Implement this!
            return 0;
        }
        virtual void write(std::ostream & out) const override {
            action->write(out);
        }
        MCTS_Environment::ActionH action;
    };
    struct InterfaceMarcObservation: public Observation {
        InterfaceMarcObservation(MCTS_Environment::ObservationH observation): observation(observation) {}
        virtual bool operator==(const Observation & other) const override {
            auto interface_observation = dynamic_cast<const InterfaceMarcObservation *>(&other);
            return interface_observation!=nullptr && *(interface_observation->observation)==*(observation);
        }
        virtual size_t get_hash() const override {
            #warning Implement this!
            return 0;
        }
        virtual void write(std::ostream & out) const override {
            observation->write(out);
        }
        MCTS_Environment::ObservationH observation;
    };
    struct InterfaceMarcState: public State {
        InterfaceMarcState(MCTS_Environment::StateH state): state(state) {}
        MCTS_Environment::StateH state;
    };

    //----members----//
    std::shared_ptr<MCTS_Environment> env_marc;

    //----methods----//
public:
    InterfaceMarc(std::shared_ptr<MCTS_Environment> env_marc): env_marc(env_marc) {}
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto interface_action = std::dynamic_pointer_cast<const InterfaceMarcAction>(action_handle);
        assert(interface_action!=nullptr);
        auto return_value = env_marc->transition(interface_action->action);
        return observation_reward_pair_t(observation_handle_t(new InterfaceMarcObservation(return_value.first)),return_value.second);
    }
    virtual action_container_t get_actions() override {
        action_container_t action_container;
        for(auto action : env_marc->get_actions()) {
            action_container.push_back(action_handle_t(new InterfaceMarcAction(action)));
        }
        return action_container;
    }
    virtual state_handle_t get_state_handle() override {
        return state_handle_t(new InterfaceMarcState(env_marc->get_state()));
    }
    virtual void set_state(const state_handle_t & state_handle) override {
        auto interface_state = std::dynamic_pointer_cast<const InterfaceMarcState>(state_handle);
        assert(interface_state!=nullptr);
        env_marc->set_state(interface_state->state);
    }
    virtual bool has_terminal_state() const override {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasTerminal);
    }
    virtual bool is_terminal_state() const override {
        return env_marc->is_terminal_state();
    }
    virtual bool is_deterministic() const override {
        return env_marc->get_info(MCTS_Environment::InfoTag::isDeterministic);
    }
    virtual bool has_max_reward() const override {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasMaxReward);
    }
    virtual reward_t max_reward() const override {
        return (reward_t)env_marc->get_info_value(MCTS_Environment::InfoTag::getMaxReward);
    }
    virtual bool has_min_reward() const override {
        return env_marc->get_info(MCTS_Environment::InfoTag::hasMinReward);
    }
    virtual reward_t min_reward() const override {
        return (reward_t)env_marc->get_info_value(MCTS_Environment::InfoTag::getMinReward);
    }
    virtual bool is_markov() const override {
        return env_marc->get_info(MCTS_Environment::InfoTag::isMarkov);
    }
};
#endif
