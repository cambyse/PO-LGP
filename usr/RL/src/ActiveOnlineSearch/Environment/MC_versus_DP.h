#ifndef MC_VERSUS_DP_H_
#define MC_VERSUS_DP_H_

#include <MCTS_Environment/IntegerEnvironment.h>

/**
 * This environment should work much better with DP backups than with MC
 * backups. */
class MC_versus_DP : public IntegerEnvironment {
    //----members----//
private:
    int action_n, depth;
    //----methods----//
public:
    MC_versus_DP(int action_n_ = 5, int depth_ = 3): action_n(action_n_), depth(depth_) {}
    virtual ~MC_versus_DP() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        double reward = 0;
        assert(abs(state)<depth);
        assert(action!=nullptr);
        //----------------------------------------------------------------------
        // The absolute value of the state corresponds to the time; the sign
        // corresponds to whether a non-zero reward is still possible (this is
        // only the case if all action were 0).
        // ----------------------------------------------------------------------
        // advance in time
        if(state>=0) ++state;
        else --state;
        // switch to "unsuccessful" if action is not 0
        if(action->action!=0 && state>0) state *= -1;
        // always give reward in successful terminal state and 50% of the time
        // otherwise
        if(abs(state)==depth && (state>0 || rand()%2==0)) reward = 1;
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)),reward);
    }
    virtual action_container_t get_actions() override {
        action_container_t ret;
        for(int a=0; a<action_n; ++a) {
            ret.push_back(action_handle_t(new IntegerAction(a)));
        }
        return ret;
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return abs(state)==depth;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual bool is_markov() const override {return false;}
    virtual void write(std::ostream & out) const override {
        out << "MC_versus_DP(action_n=" << action_n << ";depth=" << depth << ")";
    }
};

#endif /* MC_VERSUS_DP_H_ */
