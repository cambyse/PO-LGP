#ifndef STOCHASTIC1D_H_
#define STOCHASTIC1D_H_

#include <MCTS_Environment/AbstractEnvironment.h>

class Stochastic1D : public AbstractEnvironment {
    //----types/classes----//
    struct Action1D: public Action {
        Action1D(int action): action(action) {}
        virtual ~Action1D() = default;
        virtual bool operator==(const Action & other) const override {
            auto action_1D = dynamic_cast<const Action1D*>(&other);
            return action_1D!=nullptr && action_1D->action==action;
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
    int action_range;        ///< action in {-action_range,...,0,...,+action_range} are possible
    int depth;               ///< number of steps until reaching a terminal state
    double pi;               ///< probability that the chosen action will actually be executed
    double rho;              ///< probability that a reward is received when reaching a terminal state
    State state = State(0,0);
    State default_state = State(0,0);
    /**
     * If true, the reward is in principle deterministic and varies linearly
     * with the terminal position. The member rho then determines drop-out
     * probability (rho is the probability for receiving a reward). If false,
     * rewards are either zero or one, the member rho is ignored and the
     * probability of receiving a reward (of magnitude 1) varies linearly with
     * the terminal position. */
    const bool deterministic_with_dropout;
    //----methods----//
public:
    Stochastic1D(int action_range_,
                 int depth_,
                 double pi_,
                 double rho_,
                 bool deterministic_with_dropout_):
        action_range(action_range_),
        depth(depth_),
        pi(pi_),
        rho(rho_),
        deterministic_with_dropout(deterministic_with_dropout_) {}
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
            state.position += action->action;
        } else {
            // random action
            state.position += (rand()%(2*action_range+1) - action_range);
        }
        if(deterministic_with_dropout) {
            // give reward in [0,1] in terminal state; reward magnitue scales
            // linearly with position; there is a certain drop-out probability
            // (non-zero reward only with probability rho)
            if(state.time==depth && drand48()<rho) {
                reward = (double)(state.position+action_range*depth)/(2*depth*action_range);
                assert(reward>=0 && reward<=1);
            }
        } else {
            // give reward of 0 or 1, probability of receiving 1 scales linearly
            // with position, starting at 0 and reaching 0.5 at the maximum so that
            // the optimal reward also has highest variance
            if(state.time==depth) {
                double unit = (double)(state.position+action_range*depth)/(2*depth*action_range);
                if(drand48()<unit/2) reward = 1;
                assert(reward==0 || reward==1);
            }
        }
        return observation_reward_pair_t(observation_handle_t(new Observation1D(state)),reward);
    }
    virtual action_container_t get_actions() override {
        action_container_t actions;
        for(int a=-action_range; a<=action_range; ++a) {
            actions.push_back(action_handle_t(new Action1D(a)));
        }
        return actions;
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
        if(deterministic_with_dropout) {
            out << "Stochastic1D(" <<
                "action_range=" << action_range << ";" <<
                "depth=" << depth << ";" <<
                "pi=" << pi << ";" <<
                "rho=" << rho << ";" <<
                "deterministic_with_dropout=" << deterministic_with_dropout << ";" <<
                ")";
        } else {
            out << "Stochastic1D(" <<
                "action_range=" << action_range << ";" <<
                "depth=" << depth << ";" <<
                "pi=" << pi << ";" <<
                "deterministic_with_dropout=" << deterministic_with_dropout << ";" <<
                ")";
        }
    }
};

#endif /* STOCHASTIC1D_H_ */
