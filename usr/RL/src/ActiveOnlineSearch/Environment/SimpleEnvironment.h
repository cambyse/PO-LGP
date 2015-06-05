#ifndef SIMPLEENVIRONMENT_H_
#define SIMPLEENVIRONMENT_H_

class SimpleEnvironment: public AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct SimpleEnvironmentAction: public Action {
        SimpleEnvironmentAction(int action): action(action) {}
        virtual bool operator==(const Action & other) const override {
            auto other_action = dynamic_cast<const SimpleEnvironmentAction *>(&other);
            return other_action!=nullptr && other_action->action==action;
        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(action);
        }
        virtual void write(std::ostream & out) const override {
            out << action;
        }
        int action;
    };
    struct SimpleEnvironmentObservation: public Observation {
        SimpleEnvironmentObservation(int observation): observation(observation) {}
        virtual bool operator==(const Observation & other) const override {
            auto other_observation = dynamic_cast<const SimpleEnvironmentObservation *>(&other);
            return other_observation!=nullptr && other_observation->observation==observation;
        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(observation);
        }
        virtual void write(std::ostream & out) const override {
            out << observation;
        }
        int observation;
    };
    struct SimpleEnvironmentState: public State {
        SimpleEnvironmentState(int state): state(state) {}
        int state;
    };

    //----members----//
    //      ...      //

    //----methods----//
public:
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto action = std::dynamic_pointer_cast<const SimpleEnvironmentAction>(action_handle);
        if(action->action==0) {
            if(rand()%2==0) state = 1;
            else state = 2;
        } else {
            state = 2;
        }
        reward_t reward = 0;
        if(state==2 || rand()%2==0) reward = 1;
        return observation_reward_pair_t(observation_handle_t(new SimpleEnvironmentObservation(state)), reward);
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new SimpleEnvironmentAction(0)),
                    action_handle_t(new SimpleEnvironmentAction(1))});
    }
    virtual state_handle_t get_state_handle() override {
        return state_handle_t(new SimpleEnvironmentState(state));
    }
    virtual void set_state(const state_handle_t & state_handle) override {
        auto state_ = std::dynamic_pointer_cast<const SimpleEnvironmentState>(state_handle);
        state = state_->state;
    }
    virtual bool has_terminal_state() const override {return true;}
    virtual bool is_terminal_state() const override {return state==1 || state==2;}
    virtual bool is_deterministic() const override {return false;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual bool is_markov() const override {return true;}
private:
    int state = 0;
};

#endif /* SIMPLEENVIRONMENT_H_ */
