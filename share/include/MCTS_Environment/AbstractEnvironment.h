#ifndef ABSTRACTENVIRONMENT_H_
#define ABSTRACTENVIRONMENT_H_

#include <vector>
#include <memory>
#include <tuple>
#include <cassert>

class AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct Action {
        struct hash {
            size_t operator()(const Action & a) const {return a.get_hash();}
        };
        virtual ~Action() = default;
        virtual size_t get_hash() const = 0;
    };
    struct Observation {
        struct hash {
            size_t operator()(const Observation & o) const {return o.get_hash();}
        };
        virtual ~Observation() = default;
        virtual bool operator==(const Observation & other) const = 0;
        virtual bool operator!=(const Observation & other) const {return !(*this==other);}
        virtual size_t get_hash() const = 0;
    };
    struct State {
        virtual ~State() = default;
    protected:
        virtual bool operator==(const State & other) const = 0;
        virtual bool operator!=(const State & other) const {return !(*this==other);}
    };
    typedef std::shared_ptr<const Action> action_handle_t;
    typedef std::shared_ptr<const Observation> observation_handle_t;
    typedef std::shared_ptr<const State> state_handle_t;
    typedef std::vector<action_handle_t> action_container_t;
    typedef double reward_t;
    typedef std::tuple<observation_handle_t,reward_t> observation_reward_pair_t;
    template<typename C>
        struct hash: public std::hash<C> {};

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

    template<class DerivedAction>
    static action_handle_t make_action_handle(const DerivedAction & action) {
        auto derived_ptr = std::make_shared<const DerivedAction>(action);
        auto base_ptr = std::dynamic_pointer_cast<const Action>(derived_ptr);
        assert(base_ptr!=nullptr);
        return base_ptr;
    }
    template<class DerivedObservation>
    static observation_handle_t make_observation_handle(const DerivedObservation & observation) {
        auto derived_ptr = std::make_shared<const DerivedObservation>(observation);
        auto base_ptr = std::dynamic_pointer_cast<const Observation>(derived_ptr);
        assert(base_ptr!=nullptr);
        return base_ptr;
    }
    template<class DerivedState>
    static state_handle_t make_state_handle(const DerivedState & state) {
        auto derived_ptr = std::make_shared<const DerivedState>(state);
        auto base_ptr = std::dynamic_pointer_cast<const State>(derived_ptr);
        assert(base_ptr!=nullptr);
        return base_ptr;
    }
};

template<>
struct AbstractEnvironment::hash<AbstractEnvironment::Action>:
public AbstractEnvironment::Action::hash {};

template<>
struct AbstractEnvironment::hash<AbstractEnvironment::Observation>:
public AbstractEnvironment::Observation::hash {};

template<>
struct AbstractEnvironment::hash<AbstractEnvironment::action_handle_t>:
public AbstractEnvironment::Action::hash {
    size_t operator()(const action_handle_t & a) const {
        return a->get_hash();
    }
};

template<>
struct AbstractEnvironment::hash<AbstractEnvironment::observation_handle_t>:
public AbstractEnvironment::Observation::hash {
    size_t operator()(const observation_handle_t & o) const {
        return o->get_hash();
    }
};

#endif /* ABSTRACTENVIRONMENT_H_ */
