#ifndef INTEGERENVIRONMENT_H_
#define INTEGERENVIRONMENT_H_

#include <MCTS_Environment/AbstractEnvironment.h>

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

#endif /* INTEGERENVIRONMENT_H_ */
