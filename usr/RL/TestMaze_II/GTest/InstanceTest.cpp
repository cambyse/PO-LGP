#include <gtest/gtest.h>

#include "../Representation/DoublyLinkedInstance.h"
#include "../Representation/ListedReward.h"

#include "../util/util.h"

#include <vector>

#define DEBUG_LEVEL 0
#include "../util/debug.h"

using std::vector;
using util::Range;

typedef AbstractAction::ptr_t action_ptr_t;
typedef AbstractObservation::ptr_t observation_ptr_t;
typedef AbstractReward::ptr_t reward_ptr_t;
typedef AbstractInstance::ptr_t ptr_t;

reward_ptr_t reward(int n) {
    return reward_ptr_t(new ListedReward({0,1,2,3,4,5,6,7,8,9,10},n));
}

namespace {

    TEST(InstanceTest, DoublyLinkedInstance) {

        // shared pointer type
        typedef std::shared_ptr<AbstractInstance> shared_ptr_t;

        // vector to check use counts
        vector<shared_ptr_t> ins_vector;

        // create sequence and check for iteration
        {
            // create sequece of doubly linked instances
            ptr_t ins = DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward(0));
            ins_vector.push_back(ins);
            for(int idx : Range(1,10)) {
                ins = ins->append(action_ptr_t(), observation_ptr_t(), reward(idx));
                ins_vector.push_back(ins);
            }

            // check iteration
            int counter = 0;
            for(auto i : ins) {
                DEBUG_OUT(1,i);
                EXPECT_EQ(counter,i->reward->get_value());
                ++counter;
            }

            // dismiss seqence
            ins->dismiss();
        }

        // check use counts
        for(shared_ptr_t i : ins_vector) {
            EXPECT_EQ(2,i.use_count());
            DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
        }
    }

}; // end namespace
