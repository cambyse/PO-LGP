#ifndef BOTTLENECKENVIRONMENT_H_
#define BOTTLENECKENVIRONMENT_H_

#include <MCTS_Environment/AbstractEnvironment.h>

#include <sstream>

class BottleneckEnvironment: public AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct BottleneckAction: public Action {
    BottleneckAction(int action): action(action) {}
        virtual bool operator==(const Action & other) const override {
            auto x = dynamic_cast<const BottleneckAction *>(&other);
            return x!=nullptr && x->action==action;
        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(action);
        }
        virtual void write(std::ostream & out) const override {
            out << action;
        }
        int action;
    };
    struct State {
        State(int branch, int step): branch(branch), step(step) {}
        bool operator==(const State & other) const {
            return branch==other.branch && step==other.step;
        }
        int branch = 0;
        int step = 0;
    };
    struct BottleneckObservation: public Observation {
    BottleneckObservation(State state): state(state) {}
        virtual bool operator==(const Observation & other) const override {
            auto x = dynamic_cast<const BottleneckObservation *>(&other);
            return x!=nullptr && x->state==state;
        }
        virtual size_t get_hash() const override {
            std::stringstream s;
            s << state.branch << " " << state.step;
            return std::hash<std::string>()(s.str());
        }
        virtual void write(std::ostream & out) const override {
            out << "(" << state.branch << "," << state.step << ")";
        }
        State state;
    };

    //----members----//
    State default_state = State(0,0);
    State state = State(0,0);
    int branching;
    int branch_length;

    //----methods----//
public:
    BottleneckEnvironment(int branching, int branch_length);
    virtual ~BottleneckEnvironment() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override;
    virtual action_container_t get_actions() override;
    virtual void make_current_state_default() override {default_state = state;}
    virtual void reset_state() override {state = default_state;}
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state.step==branch_length;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual bool is_markov() const override {return true;}
};

#endif /* BOTTLENECKENVIRONMENT_H_ */
