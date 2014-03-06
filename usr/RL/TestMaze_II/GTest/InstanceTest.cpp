#include <gtest/gtest.h>

#include "../Representation/DoublyLinkedInstance.h"

#include <vector>

#define DEBUG_LEVEL 0
#include "../util/debug.h"

using std::vector;

typedef AbstractAction::ptr_t action_ptr_t;
typedef AbstractObservation::ptr_t observation_ptr_t;
typedef AbstractReward::ptr_t reward_ptr_t;
typedef AbstractInstance::shared_ptr_t instance_ptr_t;

namespace {

    TEST(InstanceTest, Test) {

        // vector<instance_ptr_t> instance_list;
        // instance_list.push_back(DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward_ptr_t()));
        // instance_list.push_back(instance_list.back()->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t()));
        // instance_list.push_back(instance_list.back()->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t()));
        // instance_list.push_back(instance_list.back()->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t()));
        // instance_list.push_back(instance_list.back()->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t()));

        // DEBUG_OUT(0,"From List:");
        // for(auto i : instance_list) {
        //     DEBUG_OUT(0,"    " << i);
        // }

        // DEBUG_OUT(0,"From Space:");
        // for(auto i : *instance_list[2]) {
        //     DEBUG_OUT(0,"    " << i);
        // }

        instance_ptr_t i0 = DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward_ptr_t());
        i0->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t());
        i0->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t());
        i0->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t());

        for(auto i : *i0) {
            DEBUG_OUT(0,i);
        }

    }

}; // end namespace
