#ifndef VARIANCE_ENVIRONMENTS_H_
#define VARIANCE_ENVIRONMENTS_H_

#include <MCTS_Environment/TemplateEnvironment.h>

class LowVarianceSubOptimalReward : public IntegerEnvironment {
    //----members----//
private:
    double sub_optimal_return;
    //----methods----//
public:
    LowVarianceSubOptimalReward(double sub_optimal_return = 0.4);
    virtual ~LowVarianceSubOptimalReward() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override;
    virtual action_container_t get_actions() override;
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state!=0;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {
        out << "LowVarianceSubOptimalReward(" << sub_optimal_return << ")";
    }
};

class DelayedLowVarianceSubOptimalReward : public IntegerEnvironment {
    //----members----//
private:
    int action_n, depth;
    double sub_optimal_return;
    //----methods----//
public:
    DelayedLowVarianceSubOptimalReward(int action_n = 2,
                                       int depth = 3,
                                       double sub_optimal_return = 0.4);
    virtual ~DelayedLowVarianceSubOptimalReward() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override;
    virtual action_container_t get_actions() override;
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return abs(state)==depth;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual bool is_markov() const override {return false;}
    virtual void write(std::ostream & out) const override {
        out << "DelayedLowVarianceSubOptimalReward("
            << "action_n=" << action_n
            << ";depth=" << depth
            << ";sub-optimal-return=" << sub_optimal_return << ")";
    }
};

#endif /* VARIANCE_ENVIRONMENTS_H_ */
