#include <gtest/gtest.h>

#include "../Representation/DoublyLinkedInstance.h"
#include "../Representation/ListedReward.h"

#include "../util/util.h"

#include <vector>
#include <set>

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using std::vector;
using std::set;
using util::Range;

typedef AbstractAction::ptr_t action_ptr_t;
typedef AbstractObservation::ptr_t observation_ptr_t;
typedef AbstractReward::ptr_t reward_ptr_t;
typedef AbstractInstance::ptr_t ptr_t;
typedef std::shared_ptr<AbstractInstance> shared_ptr_t;

reward_ptr_t reward(int n) {
    return reward_ptr_t(new ListedReward({0,1,2,3,4,5,6,7,8,9,10},n));
}

// Create a sequence like
//
//    <--7==8==9==10               //
//               \                 //
//                V                //
//    <--0==1==2==3==4==5==6-->    //
//
// and return instance nr 10 (== means the instances are linked forwards AND
// backwards, --> are singlelinks). Backwards iteration is expected run from 10
// down to 0, forwards iteration should then run from 0 to 7.
ptr_t create_test_sequence(set<shared_ptr_t> * ins_set = nullptr) {
    #warning does not produce the above sequence
    ptr_t ins3;
    ptr_t ins = DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward(0));
    if(ins_set) ins_set->insert(ins);
    for(int i : Range(1,7)) {
        ins = ins->append(action_ptr_t(), observation_ptr_t(), reward(i));
        if(ins_set) ins_set->insert(ins);
        if(i==3) {
            ins3 = ins;
        }
    }
    ins = DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward(8), ins3, AbstractInstance::create_invalid());
    if(ins_set) ins_set->insert(ins);
    for(int i : Range(9,10)) {
        ins = ins->append(action_ptr_t(), observation_ptr_t(), reward(i));
        if(ins_set) ins_set->insert(ins);
    }
    return ins;
}

namespace {

    TEST(InstanceTest, Iteration) {
        ptr_t ins = create_test_sequence();
        EXPECT_EQ(10,ins->reward->get_value());
        EXPECT_EQ(8,ins->prev(2)->reward->get_value());
        EXPECT_EQ(0,ins->prev(6)->reward->get_value());
        EXPECT_EQ(7,ins->prev(6)->next(7)->reward->get_value());
    }

    TEST(InstanceTest, DestroySequence) {

        set<shared_ptr_t> ins_set;

        // create and destroy sequence from nr 10
        create_test_sequence(&ins_set)->destroy_sequence();

        // check use counts
        for(shared_ptr_t i : ins_set) {
            EXPECT_EQ(2,i.use_count());
            DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
        }

        ins_set.clear();

        // create and destroy sequence from nr 0
        create_test_sequence(&ins_set)->prev(6)->destroy_sequence();

        // check use counts
        for(shared_ptr_t i : ins_set) {
            switch((int)i->reward->get_value()) {
            case 3:
            case 8:
            case 10:
                EXPECT_EQ(3,i.use_count());
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
                break;
            case 9:
                EXPECT_EQ(4,i.use_count());
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
                break;
            default:
                EXPECT_EQ(2,i.use_count());
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
            }
        }
    }

}; // end namespace
