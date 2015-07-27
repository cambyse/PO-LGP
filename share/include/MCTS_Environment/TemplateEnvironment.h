#ifndef TEMPLATEENVIRONMENT_H_
#define TEMPLATEENVIRONMENT_H_

#include <MCTS_Environment/AbstractEnvironment.h>

class TemplateEnvironment: public AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct TemplateAction: public Action {
        virtual bool operator==(const Action & other) const override;
        virtual size_t get_hash() const override;
        virtual void write(std::ostream &) const override;
    };
    struct TemplateObservation: public Observation {
        virtual bool operator==(const Observation & other) const override;
        virtual size_t get_hash() const override;
        virtual void write(std::ostream &) const override;
    };

    //----members----//
    //      ...      //

    //----methods----//
public:
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override;
    virtual action_container_t get_actions() override;
    virtual void make_current_state_default() override;
    virtual void reset_state() override;
    virtual bool has_terminal_state() const override;
    virtual bool is_terminal_state() const override;
    virtual bool is_deterministic() const override;
    virtual bool has_max_reward() const override;
    virtual reward_t max_reward() const override;
    virtual bool has_min_reward() const override;
    virtual reward_t min_reward() const override;
    virtual bool is_markov() const override;
    virtual void write(std::ostream & out) const override;
};

class IntegerEnvironment: public AbstractEnvironment {
public:
    struct IntegerAction: public Action {
        IntegerAction(int action): action(action) {}
        virtual ~IntegerAction() = default;
        virtual bool operator==(const Action & other) const override {
            auto integer_action = dynamic_cast<const IntegerAction*>(&other);
            return integer_action!=nullptr && integer_action->action==action;
        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(action);
        }
        virtual void write(std::ostream & out) const override {
            out << action;
        }
        int action;
    };
    struct IntegerObservation: public Observation {
        IntegerObservation(int observation): observation(observation) {}
        virtual ~IntegerObservation() = default;
        virtual bool operator==(const Observation & other) const override {
            auto integer_observation = dynamic_cast<const IntegerObservation*>(&other);
            return integer_observation!=nullptr && integer_observation->observation==observation;
        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(observation);
        }
        virtual void write(std::ostream & out) const override {
            out << observation;
        }
        int observation;
    };
public:
    IntegerEnvironment() = default;
    virtual ~IntegerEnvironment() = default;
    virtual void make_current_state_default() override final {default_state = state;}
    virtual void reset_state() override final {state = default_state;}
    virtual bool is_markov() const override {return true;}
    virtual void write(std::ostream & out) const override = 0;
protected:
    int state = 0;
    int default_state = 0;
};

#endif /* TEMPLATEENVIRONMENT_H_ */
