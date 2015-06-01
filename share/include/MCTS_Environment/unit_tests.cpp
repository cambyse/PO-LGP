#include <gtest/gtest.h>

#include <iostream>

#include "AbstractFiniteEnvironment.h"

#include <unordered_map>
#include <unordered_set>

using namespace std;

class TestEnvironment: public AbstractFiniteEnvironment<int,int> {
public:
    TestEnvironment(): AbstractFiniteEnvironment<int,int>({0,1},{0,1}) {}
    virtual ~TestEnvironment() = default;
    virtual state_reward_pair_t finite_transition(const state_t & state,
                                                  const action_t & action) const override {
        if(action==0) {
            return state_reward_pair_t(state,0);
        } else {
            return state_reward_pair_t((state+1)%2,1);
        }
    }
    bool has_terminal_state() const override {return false;}
    bool is_terminal_state() const override {return false;}
    virtual bool is_deterministic() const override {return true;}
    virtual bool has_max_reward() const override {return true;}
    virtual reward_t max_reward() const override {return 1;}
    virtual bool has_min_reward() const override {return true;}
    virtual reward_t min_reward() const override {return 0;}
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
        action_handle_t other_action = make_shared<const TestEnvironment::action_t>(
            *dynamic_pointer_cast<const TestEnvironment::action_t>(action)
            );

        action_set.insert(other_action);
        observation_set.insert(observation);
        EXPECT_LE(action_set.size(),2);
        EXPECT_LE(observation_set.size(),2);
    }
}

TEST(AbstractEnvironment, OstremOperator) {
    std::shared_ptr<AbstractEnvironment> env(new TestEnvironment);
    typedef AbstractEnvironment::action_handle_t action_handle_t;
    typedef AbstractEnvironment::state_handle_t state_handle_t;
    typedef AbstractEnvironment::observation_handle_t observation_handle_t;

    state_handle_t state = env->get_state_handle();
    auto action_list = env->get_actions();
    for(action_handle_t action : action_list) {
        auto observation_reward = env->transition(action);
        observation_handle_t observation = get<0>(observation_reward);
        //cout << "Action " << *action << " / Observation " << *observation << endl;
        env->set_state(state);
    }
}
