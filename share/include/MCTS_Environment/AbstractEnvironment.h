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
      virtual ~Action(){}
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
      virtual ~Observation(){}
        virtual bool operator==(const Observation & other) const = 0;
        virtual bool operator!=(const Observation & other) const {return !(*this==other);}
        virtual size_t get_hash() const = 0;
        friend std::ostream& operator<<(std::ostream & out, const Observation & observation) {
            observation.write(out);
            return out;
        }
        virtual void write(std::ostream &) const = 0;
    };
    typedef std::shared_ptr<const Action> action_handle_t;
    typedef std::shared_ptr<const Observation> observation_handle_t;
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
    virtual ~AbstractEnvironment(){}
    /**
     * Perform a transition by executing the given action and return the
     * resulting observation and reward. */
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) = 0;
    /**
     * Get the available actions in the current state. \warning Note that the
     * returned action container may be constructed ad-hoc so writing @code
     for(auto action_it = environment.get_actions().begin(); action_it!=environment.get_actions().end(); ++action_it) {
         ...
     }
     * @endcode is not safe, whereas @code
     auto actions = environment.get_actions();
     for(auto action_it = actions().begin(); action_it!=actions().end(); ++action_it) {
         ...
     }
     * @endcode or @code
     for(auto action : environment.get_actions()) {
         ...
     }
     * @endcode is. */
    virtual action_container_t get_actions() = 0;
    /**
     * Make the current state default. reste_state() will then reset to this
     * state. */
    virtual void make_current_state_default() = 0;
    /**
     * Resets the environment to the default state. */
    virtual void reset_state() = 0;
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
    /**
     * Write a description of the environment. This is not the state of the
     * environment but rather something like an identifier, e.g., the class name
     * with relevant parameters defining the properties of the environment. */
    virtual void write(std::ostream & out) const {out<<"AbstractEnvironment()";}
    /**
     * Use write() method to define << operator. */
    friend std::ostream& operator<<(std::ostream & out, const AbstractEnvironment & environment) {
        environment.write(out);
        return out;
    }
};

#endif /* ABSTRACTENVIRONMENT_H_ */
