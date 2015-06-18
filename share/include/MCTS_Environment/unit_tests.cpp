#include <gtest/gtest.h>

#include <iostream>

#include <unordered_map>
#include <unordered_set>

#include <MCTS_Environment/AbstractEnvironment.h>

using namespace std;

class TestEnvironment: public AbstractEnvironment {
    //----typedefs/classes----//
public:
    struct TestAction: public Action {
        TestAction(int action): action(action) {}
        virtual bool operator==(const Action & other) const override {
            auto a = dynamic_cast<const TestAction *>(&other);
            return a!=nullptr && a->action==action;
        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(action);
        }
        virtual void write(std::ostream & out) const override {
            out << action;
        }
        int action;
    };
    struct TestObservation: public Observation {
        TestObservation(int observation): observation(observation) {}
        virtual bool operator==(const Observation & other) const override {
            auto o = dynamic_cast<const TestObservation *>(&other);
            return o!=nullptr && o->observation==observation;

        }
        virtual size_t get_hash() const override {
            return std::hash<int>()(observation);
        }
        virtual void write(std::ostream & out) const override {
            out << observation;
        }
        int observation;
    };
    //----members----//
    int state = 0;
    int default_state = 0;
    //----methods----//
public:
    virtual observation_reward_pair_t transition(const action_handle_t & action_handle) override {
        auto action = std::dynamic_pointer_cast<const TestAction>(action_handle);
        EXPECT_NE(action,nullptr);
        state = action->action;
        return observation_reward_pair_t(observation_handle_t(new TestObservation(state)), state);
    }
    virtual action_container_t get_actions() override {
        return action_container_t({action_handle_t(new TestAction(0)),
                    action_handle_t(new TestAction(1))});
    }
    virtual void make_current_state_default() override {default_state = state;}
    virtual void reset_state() override {state = default_state;}
    virtual bool has_terminal_state() const override {return false;}
    virtual bool is_terminal_state() const override {return false;}
    virtual bool is_deterministic() const override {return true;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
    virtual bool is_markov() const override {return true;}
};

TEST(AbstractEnvironment, UnorderedSets) {
    std::shared_ptr<AbstractEnvironment> env(new TestEnvironment);
    typedef AbstractEnvironment::action_handle_t action_handle_t;
    typedef AbstractEnvironment::observation_handle_t observation_handle_t;
    unordered_set<action_handle_t,
                  AbstractEnvironment::ActionHash,
                  AbstractEnvironment::ActionEq> action_set;
    unordered_set<observation_handle_t,
                  AbstractEnvironment::ObservationHash,
                  AbstractEnvironment::ObservationEq> observation_set;
    for(int i=0; i<100; ++i) {
        auto action_list = env->get_actions();
        action_handle_t action = action_list[rand()%action_list.size()];
        auto observation_reward = env->transition(action);
        observation_handle_t observation = get<0>(observation_reward);

        // this makes a copy of the underlying action object to reveal whether
        // the ActionHash and ActionEq actually do their job
        action_handle_t other_action = std::make_shared<const TestEnvironment::TestAction>(
            *dynamic_pointer_cast<const TestEnvironment::TestAction>(action)
            );

        action_set.insert(other_action);
        observation_set.insert(observation);
        EXPECT_LE(action_set.size(),2);
        EXPECT_LE(observation_set.size(),2);
    }
}

TEST(AbstractEnvironment, OstreamOperator) {
    std::shared_ptr<AbstractEnvironment> env(new TestEnvironment);
    typedef AbstractEnvironment::action_handle_t action_handle_t;
    typedef AbstractEnvironment::observation_handle_t observation_handle_t;

    auto action_list = env->get_actions();
    for(action_handle_t action : action_list) {
        auto observation_reward = env->transition(action);
        observation_handle_t observation = get<0>(observation_reward);
        //cout << "Action " << *action << " / Observation " << *observation << endl;
        env->reset_state();
    }
}
