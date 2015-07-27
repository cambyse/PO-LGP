#ifndef VARIANCE_ENVIRONMENTS_H_
#define VARIANCE_ENVIRONMENTS_H_

#include <MCTS_Environment/TemplateEnvironment.h>

#include <util/debug.h>

class DelayedLowVarianceSubOptimalReward : public IntegerEnvironment {
    //----members----//
private:
    double sub_optimal_return;
    //----methods----//
public:
    DelayedLowVarianceSubOptimalReward(double sub_optimal_return_ = 0.4): sub_optimal_return(sub_optimal_return_) {}
    virtual ~DelayedLowVarianceSubOptimalReward() = default;
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto action = std::dynamic_pointer_cast<const IntegerAction>(action_handle);
        double reward = 0;
        DEBUG_EXPECT(state>=0);
        DEBUG_EXPECT(state<3);
        DEBUG_EXPECT(action!=nullptr);
        DEBUG_EXPECT(state!=0 || (action->action==1 || action->action==2));
        DEBUG_EXPECT(state==0 || action->action==0);
        switch(state) {
        case 0:
            state=action->action;
            break;
        case 1:
            // low-variance sub-optimal return
            reward = sub_optimal_return;
            state = 3;
            break;
        case 2:
            // high-variance optimal return
            if(rand()%2==0) {
                reward = 1;
            } else {
                reward = 0;
            }
            state = 3;
            break;
        default:
            DEBUG_DEAD_LINE;
        }
        return observation_reward_pair_t(observation_handle_t(new IntegerObservation(state)),reward);
    }
    virtual action_container_t get_actions() override {
        if(state==0) {
            return action_container_t({action_handle_t(new IntegerAction(1)),
                        action_handle_t(new IntegerAction(2))});
        } else {
            return action_container_t({action_handle_t(new IntegerAction(0))});
        }
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state==3;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual void write(std::ostream & out) const override {
        out << "DelayedLowVarianceSubOptimalReward(" << sub_optimal_return << ")";
    }
};

#include <util/debug_exclude.h>

#endif /* VARIANCE_ENVIRONMENTS_H_ */
