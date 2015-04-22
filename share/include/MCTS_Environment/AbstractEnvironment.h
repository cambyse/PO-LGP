#ifndef ABSTRACTENVIRONMENT_H_
#define ABSTRACTENVIRONMENT_H_

#include <vector>
#include <memory>
#include <tuple>

class AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct Action {
        virtual ~Action() = default;
    };
    struct Observation {
        virtual ~Observation() = default;
        virtual bool operator==(const Observation & other) const = 0;
        virtual bool operator!=(const Observation & other) const {return !*this==other;}
    };
    struct State {
        virtual ~State() = default;
        virtual bool operator==(const State & other) const = 0;
        virtual bool operator!=(const State & other) const {return !*this==other;}
    };
    typedef std::shared_ptr<const Action> ActionHandle;
    typedef std::shared_ptr<const Observation> ObservationHandle;
    typedef std::shared_ptr<const State> StateHandle;
    typedef std::vector<ActionHandle> ActionContainer;
    typedef double reward_t;
    typedef std::tuple<ObservationHandle,reward_t> observation_reward_pair_t;

    //----methods----//
public:
    AbstractEnvironment() = default;
    virtual ~AbstractEnvironment() = default;
    /**
     * Perform a transition by execution the given action and return the
     * resulting observation and reward. */
    virtual observation_reward_pair_t transition(const ActionHandle & action) = 0;
    /**
     * Get the available actions in the current state. */
    virtual const ActionContainer get_actions() = 0;
    /**
     * Get the current state. */
    virtual const StateHandle get_state_handle() = 0;
    /**
     * Set the environment's state the the given state. */
    virtual const StateHandle set_state(const StateHandle & state_handle) = 0;
    /**
     * Return whether the environment has a terminal state. */
    virtual bool has_terminal_state() const = 0;
    /**
     * Return whether the current state is a terminal state. */
    virtual bool is_terminal_state() const = 0;
    /**
     * Return whether the environment has deterministic transitions and rewards. */
    virtual bool is_deterministic() const = 0;
    /**
     * Return whether the environment has a maximum reward. */
    virtual bool has_max_reward() const = 0;
    /**
     * Return the maximum reward. \warning If has_max_reward() returns \p false
     * the return value of this function is undefined. */
    virtual reward_t max_reward() const = 0;
    /**
     * Return whether the environment has a minimum reward. */
    virtual bool has_min_reward() const = 0;
    /**
     * Return the minimum reward. \warning If has_min_reward() returns \p false
     * the return value of this function is undefined. */
    virtual reward_t min_reward() const = 0;
    /**
     * Return whether the environment is Markov. */
    virtual bool is_markov() const = 0;
};

#endif /* ABSTRACTENVIRONMENT_H_ */
