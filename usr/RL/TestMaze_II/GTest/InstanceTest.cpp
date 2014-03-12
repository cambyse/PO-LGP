#include <gtest/gtest.h>

#include "../Representation/DoublyLinkedInstance.h"
#include "../Representation/ListedReward.h"

#include "../util/util.h"

#include <vector>
#include <set>

#define DEBUG_LEVEL 2
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

/** \brief Create an example structure to work with
 *
 * Create a structure like
 *
 * @code
 *    <--0==1==2==3.
 *                  \
 *                   V
 *    <--4==5==6==7==8==9==10-->
 * @endcode
 *
 * and return instance nr 3 (== means the instances are linked forwards AND
 * backwards, --> are single links). Forwards iteration is expected run from 0
 * over 3 and 8 to 10, backwards iteration should then run from 10 down to 4. */
ptr_t create_test_sequence(set<shared_ptr_t> * ins_set = nullptr) {
    DEBUG_OUT(2,"Building test sequence...");
    ptr_t ins8;
    ptr_t ins = DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward(4));
    if(ins_set) ins_set->insert(ins);
    for(int i : Range(5,10)) {
        ins = ins->append(action_ptr_t(), observation_ptr_t(), reward(i));
        if(ins_set) ins_set->insert(ins);
        if(i==8) {
            ins8 = ins;
        }
    }
    ins = DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward(0));
    if(ins_set) ins_set->insert(ins);
    for(int i : Range(1,3)) {
        ins = ins->append(action_ptr_t(), observation_ptr_t(), reward(i));
        if(ins_set) ins_set->insert(ins);
    }
    ins->set_successor(ins8);
    DEBUG_OUT(2,"...DONE");
    return ins;
}

namespace {

    TEST(InstanceTest, Minimal) {
        {
            ptr_t ins = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(0));
            ins = ins->append(action_ptr_t(),observation_ptr_t(),reward(1));
            ins = ins->append(action_ptr_t(),observation_ptr_t(),reward(2));
            ins->detach();
        }
        EXPECT_EQ(0,AbstractInstance::memory_check());
    }

    TEST(InstanceTest, Loop) {
        ptr_t ins0 = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(0));
        ptr_t ins1 = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(1));
        ins0->set_predecessor(ins1);
        ins0->set_successor(ins1);
        ins1->set_predecessor(ins0);
        ins1->set_successor(ins0);
        // forwards loop
        {
            int counter = 0;
            for(auto ins = ins1->next(); counter<10; ++counter, ins=ins->next()) {
                EXPECT_EQ(counter%2,ins->reward->get_value());
            }
        }
        // backwards loop
        {
            int counter = 0;
            for(auto ins = ins1->prev(); counter<10; ++counter, ins=ins->prev()) {
                EXPECT_EQ(counter%2,ins->reward->get_value());
            }
        }
        ins0->detach();
        ins1->detach();
    }

    TEST(InstanceTest, Iteration) {
        ptr_t ins = create_test_sequence();
        EXPECT_EQ(3,ins->reward->get_value());
        EXPECT_EQ(0,ins->prev(3)->reward->get_value());
        EXPECT_EQ(8,ins->next(1)->reward->get_value());
        EXPECT_EQ(10,ins->next(3)->reward->get_value());
        EXPECT_EQ(4,ins->next(1)->prev(4)->reward->get_value());
        EXPECT_EQ(10,ins->next(1)->prev(4)->next(6)->reward->get_value());
    }

    TEST(InstanceTest, Detach) {

        set<shared_ptr_t> ins_set;

        // create and detach nr 6
        EXPECT_EQ(1,create_test_sequence(&ins_set)->next()->prev(2)->detach());

        // check use counts
        for(shared_ptr_t i : ins_set) {
            DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
            switch((int)i->reward->get_value()) {
            case 6:
                EXPECT_EQ(2,i.use_count());
                break;
            case 8:
                EXPECT_EQ(5,i.use_count());
                break;
            case 1:
            case 2:
            case 9:
                EXPECT_EQ(4,i.use_count());
                break;
            default:
                EXPECT_EQ(3,i.use_count());
            }
        }
    }

    TEST(InstanceTest, DetachReachable) {
        {
            set<shared_ptr_t> ins_set;

            // create and detach reachable from nr 3
            EXPECT_EQ(11,create_test_sequence(&ins_set)->detach_reachable());

            // check use counts
            for(shared_ptr_t i : ins_set) {
                EXPECT_EQ(2,i.use_count());
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
            }
        }
        {
            set<shared_ptr_t> ins_set;

            // create and detach reachable from nr 6
            EXPECT_EQ(7,create_test_sequence(&ins_set)->next()->prev(2)->detach_reachable());

            // check use counts
            for(shared_ptr_t i : ins_set) {
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
                switch((int)i->reward->get_value()) {
                case 3:
                    EXPECT_EQ(i->next(),*(i->end()));
                case 0:
                    EXPECT_EQ(3,i.use_count());
                    break;
                case 1:
                case 2:
                    EXPECT_EQ(4,i.use_count());
                    break;
                default:
                    EXPECT_EQ(2,i.use_count());
                }
            }
        }
    }

}; // end namespace
