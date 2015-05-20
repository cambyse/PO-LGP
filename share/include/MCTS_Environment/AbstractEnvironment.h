#ifndef ABSTRACTENVIRONMENT_H_
#define ABSTRACTENVIRONMENT_H_

#include <vector>
#include <memory>
#include <tuple>
#include <cassert>
#include <ostream>

class AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct Action {
        virtual ~Action() = default;
        virtual bool operator==(const Action & other) const = 0;
        virtual bool operator!=(const Action & other) const {return !(*this==other);}
        virtual size_t get_hash() const = 0;
        friend std::ostream& operator<<(std::ostream & out, const Action & action) {
            action.write(out);
            return out;
        }
        virtual void write(std::ostream &) const = 0;
    };
    struct Observation {
        virtual ~Observation() = default;
        virtual bool operator==(const Observation & other) const = 0;
        virtual bool operator!=(const Observation & other) const {return !(*this==other);}
        virtual size_t get_hash() const = 0;
        friend std::ostream& operator<<(std::ostream & out, const Observation & observation) {
            observation.write(out);
            return out;
        }
        virtual void write(std::ostream &) const = 0;
    };
    struct State {
        virtual ~State() = default;
    };
    typedef std::shared_ptr<const Action> action_handle_t;
    typedef std::shared_ptr<const Observation> observation_handle_t;
    typedef std::shared_ptr<const State> state_handle_t;
    typedef std::vector<action_handle_t> action_container_t;
    typedef double reward_t;
    typedef std::tuple<observation_handle_t,reward_t> observation_reward_pair_t;
    struct ActionHash {
        size_t operator()(const action_handle_t & action) const {
            return action->get_hash();
        }
    };
    struct ObservationHash {
        size_t operator()(const observation_handle_t & observation) const {
            return observation->get_hash();
        }
    };
    struct ActionEq {
        size_t operator()(const action_handle_t & action1,
                          const action_handle_t & action2) const {
            return *(action1)==*(action2);
        }
    };
    struct ObservationEq {
        size_t operator()(const observation_handle_t & observation1,
                          const observation_handle_t & observation2) const {
            return *(observation1)==*(observation2);
        }
    };

    //----methods----//
public:
    AbstractEnvironment() = default;
    virtual ~AbstractEnvironment() = default;
    /**
     * Perform a transition by execution the given action and return the
     * resulting observation and reward. */
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) = 0;
    /**
     * Perform a transition from the given state without actually changing the environment. */
    virtual observation_reward_pair_t transition(const state_handle_t & state_handle,
                                                 const action_handle_t & action_handle) {
        auto current_state = get_state_handle();
        this->set_state(state_handle);
        auto return_value = transition(action_handle);
        this->set_state(current_state);
        return return_value;
    }
    /**
     * Get the available actions in the current state. */
    virtual action_container_t get_actions() = 0;
    /**
     * Get the current state. */
    virtual state_handle_t get_state_handle() = 0;
    /**
     * Set the environment's state the the given state. */
    virtual void set_state(const state_handle_t & state_handle) = 0;
    /**
     * Return whether the environment has a terminal state. */
    virtual bool has_terminal_state() const = 0;
    /**
     * Return whether the current state is a terminal state. */
    virtual bool is_terminal_state() const = 0;
    /**
     * Return whether the given state is a terminal state. */
    virtual bool is_terminal_state(const state_handle_t & state_handle) {
        auto current_state = get_state_handle();
        this->set_state(state_handle);
        auto return_value = is_terminal_state();
        this->set_state(current_state);
        return return_value;
    }
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
