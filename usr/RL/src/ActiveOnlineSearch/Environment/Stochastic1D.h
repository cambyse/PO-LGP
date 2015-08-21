#ifndef STOCHASTIC1D_H_
#define STOCHASTIC1D_H_

#include <MCTS_Environment/AbstractEnvironment.h>

class Stochastic1D : public AbstractEnvironment {
    //----types/classes----//
    enum ACTION { UP, DOWN };
    struct Action1D: public Action {
        Action1D(ACTION action): action(action) {}
        virtual ~Action1D() = default;
        virtual bool operator==(const Action & other) const override {
            auto action_1D = dynamic_cast<const Action1D*>(&other);
            return action_1D!=nullptr && action_1D->action==action;
        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(action);
        }
        virtual void write(std::ostream & out) const override {
            switch(action) {
            case UP:
                out << "UP";
                break;
            case DOWN:
                out << "DOWN";
                break;
            }
        }
        ACTION action;
    };
    struct State {
        State(int time_, int position_): time(time_), position(position_) {}
        int time, position;
        bool operator==(const State & other) const {
            return time==other.time && position==other.position;
        }
    };
    struct Observation1D: public Observation {
        Observation1D(State state): state(state) {}
        virtual ~Observation1D() = default;
        virtual bool operator==(const Observation & other) const override {
            auto observation_1D = dynamic_cast<const Observation1D*>(&other);
            return observation_1D!=nullptr && observation_1D->state==state;
        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(state.position);
        }
        virtual void write(std::ostream & out) const override {
            out << "(t=" << state.time << ",pos=" << state.position << ")";
        }
        State state;
    };
    //----members----//
protected:
    int depth;               ///< number of steps until reaching a terminal state
    double pi;               ///< probability that the chosen action will actually be executed
    State state = State(0,0);
    State default_state = State(0,0);
    //----methods----//
public:
    Stochastic1D(int depth_ = 10, double pi_ = 0.6): depth(depth_), pi(pi_) {}
    virtual ~Stochastic1D() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto action = std::dynamic_pointer_cast<const Action1D>(action_handle);
        double reward = 0;
        assert(abs(state.time)<depth);
        assert(action!=nullptr);
        // advance in time
        ++state.time;
        // do intended action with probability pi
        if(drand48()<pi) {
            // intendet action
            switch(action->action) {
            case UP:
                ++state.position;
                break;
            case DOWN:
                --state.position;
                break;
            }
        } else {
            // switched action
            switch(action->action) {
            case UP:
                --state.position;
                break;
            case DOWN:
                ++state.position;
                break;
            }
        }
        // give deterministic reward in [0,1] in terminal state; reward magnitue
        // scales linearly with position
        if(state.time==depth) {
            reward = (double)(state.position+depth)/(2*depth);
            assert(reward>=0 && reward<=1);
        }
        return observation_reward_pair_t(observation_handle_t(new Observation1D(state)),reward);
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new Action1D(UP)),
                                   action_handle_t(new Action1D(DOWN))});
    }
    virtual void make_current_state_default() override {default_state = state;}
    virtual void reset_state() override final {state = default_state;}
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return abs(state.time)==depth;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual bool is_markov() const override {return true;}
    virtual void write(std::ostream & out) const override {
        out << "Stochastic1D(depth=" << depth << ";pi=" << pi << ")";
    }
};

#endif /* STOCHASTIC1D_H_ */
