#pragma once

#include <vector>
#include <memory>
#include <tuple>

/** This is an abstraction of an environment for MCTS. The environment essentially only needs to simulate (=transition
 *  forward the state for given actions), and for each transition return the observation and reward. Additionally, it
 *  needs to provide the set of feasible decisions for the current state. For the MCTS solver, states, actions, and
 *  rewards are fully abstract entities -- they can only be referred to via 'handles'. */
struct MCTS_Environment{
    /** A generic State-Action-Observation object as abstraction of a real state, action, or observation. The environment can,
     *  via dynamic casting, get the semantics back. The MCTS solver should not use any other properties than equality. */
    struct SAO {
      virtual ~SAO() = default;
      virtual bool operator==(const SAO & other) const { return true; } ///< why true? Because a NIL object (e.g. non-observation) is always equal to NIL
      virtual bool operator!=(const SAO & other) const { return !(*this==other); }
      virtual void write(ostream& os) const { os <<"NIL_SAO"; }
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
