#include <gtest/gtest.h>

#include "../Representation/DoublyLinkedInstance.h"

#include <vector>

#define DEBUG_LEVEL 0
#include "../util/debug.h"

using std::vector;

typedef AbstractAction::ptr_t action_ptr_t;
typedef AbstractObservation::ptr_t observation_ptr_t;
typedef AbstractReward::ptr_t reward_ptr_t;
typedef std::shared_ptr<AbstractInstance> instance_ptr_t;

namespace {

    TEST(InstanceTest, Test) {

        vector<instance_ptr_t> instance_list;
        instance_list.push_back(instance_ptr_t(new DoublyLinkedInstance(action_ptr_t(), observation_ptr_t(), reward_ptr_t())));
        instance_list.push_back(instance_list.back()->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t())->get_self_ptr());
        instance_list.push_back(instance_list.back()->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t())->get_self_ptr());
        instance_list.push_back(instance_list.back()->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t())->get_self_ptr());
        instance_list.push_back(instance_list.back()->append(action_ptr_t(), observation_ptr_t(), reward_ptr_t())->get_self_ptr());

        DEBUG_OUT(0,"From List:");
        for(auto i : instance_list) {
            DEBUG_OUT(0,"    " << i);
        }

        DEBUG_OUT(0,"From Space:");
        for(auto i : *(instance_list[2])) {
            DEBUG_OUT(0,"    " << i);
        }

    }

}; // end namespace
