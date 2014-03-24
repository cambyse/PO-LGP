#include <gtest/gtest.h>

#include "../Representation/DoublyLinkedInstance.h"
#include "../Representation/ListedReward.h"

#include "../util/util.h"

#include <vector>
#include <set>

#define DEBUG_LEVEL 0
#include "../util/debug.h"

using std::vector;
using std::set;
using util::Range;
using util::INVALID;

typedef AbstractAction::ptr_t action_ptr_t;
typedef AbstractObservation::ptr_t observation_ptr_t;
typedef AbstractReward::ptr_t reward_ptr_t;
typedef AbstractInstance::ptr_t ptr_t;
typedef AbstractInstance::const_ptr_t const_ptr_t;
typedef std::shared_ptr<AbstractInstance> shared_ptr_t;
typedef std::shared_ptr<const AbstractInstance> shared_const_ptr_t;

static int do_memory_check = -1;

#define MEMORY_CHECK {                                                  \
        if(do_memory_check==-1) {                                       \
            do_memory_check = AbstractInstance::memory_check_request()?1:0; \
            if(do_memory_check==0) {                                    \
                DEBUG_WARNING("Memory check disabled");                 \
            }                                                           \
        }                                                               \
        if(do_memory_check==1) {                                        \
            EXPECT_EQ(0,AbstractInstance::memory_check()) << "Note: This may be caused by other tests"; \
        }                                                               \
    }

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
    ins->set_non_const_successor(ins8);
    DEBUG_OUT(2,"...DONE");
    return ins;
}

namespace {

    TEST(InstanceTest, Minimal) {
        {
            DEBUG_OUT(2,"Create...");
            ptr_t ins = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(0));
            DEBUG_OUT(2,"DONE");
            DEBUG_OUT(2,"Append...");
            ins = ins->append(action_ptr_t(),observation_ptr_t(),reward(1));
            DEBUG_OUT(2,"DONE");
            DEBUG_OUT(2,"Append...");
            ins = ins->append(action_ptr_t(),observation_ptr_t(),reward(2));
            DEBUG_OUT(2,"DONE");
            DEBUG_OUT(2,"Detach...");
            EXPECT_EQ(3,ins->detach_reachable());
            DEBUG_OUT(2,"DONE");
        }
        MEMORY_CHECK;
    }

    TEST(InstanceTest, Reconnecting) {
        {
            ptr_t ins0 = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(0));
            ptr_t ins1 = ins0->append(action_ptr_t(),observation_ptr_t(),reward(1));
            ptr_t ins2 = ins1->append(action_ptr_t(),observation_ptr_t(),reward(2));
            ptr_t ins3 = ins2->append(action_ptr_t(),observation_ptr_t(),reward(3));
            ptr_t ins4 = ins3->append(action_ptr_t(),observation_ptr_t(),reward(4));

            ptr_t ins5 = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(5),const_ptr_t(ins3),INVALID);
            ptr_t ins6 = ins5->append(action_ptr_t(),observation_ptr_t(),reward(6));
            ptr_t ins7 = ins6->append(action_ptr_t(),observation_ptr_t(),reward(7));

            ptr_t ins8 = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(8),const_ptr_t(ins6),const_ptr_t(ins3));
            ins8->set_const_successor(ins4);

            ins8->detach_reachable();
        }
        MEMORY_CHECK;
    }

    TEST(InstanceTest, Equality) {
        ptr_t ins1_a = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(1));
        ptr_t ins1_b = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(1));
        ptr_t ins2_a = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(2));
        ptr_t ins2_b = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(2));
        //----------------------------//
        // compare isolated instances //
        //----------------------------//
        EXPECT_EQ(ins1_a,ins1_b);
        EXPECT_EQ(ins2_a,ins2_b);
        EXPECT_NE(ins1_a,ins2_a);
        EXPECT_NE(ins1_b,ins2_b);
        EXPECT_NE(ins1_a,ins2_b);
        EXPECT_NE(ins1_b,ins2_a);
        //-----------------------------//
        // compare connected instances //
        //-----------------------------//
        ins1_a->set_non_const_successor(ins2_a);
        ins1_b->set_non_const_successor(ins2_a);
        // should be equal since they have equal successor
        EXPECT_EQ(ins1_a,ins1_b);
        ins1_b->set_non_const_successor(ins2_b);
        // should be unequal since they have unequal successor
        EXPECT_NE(ins1_a,ins1_b);
        // although their successors compare equal
        EXPECT_EQ(ins1_a->non_const_next(),ins1_b->non_const_next());
    }

    TEST(InstanceTest, Invalid) {
        {
            DEBUG_OUT(2,"ins_abs...");
            ptr_t ins_abs = AbstractInstance::create(action_ptr_t(), observation_ptr_t(), reward_ptr_t());
            DEBUG_OUT(2,"check");
            EXPECT_EQ(INVALID,ins_abs);
        }
        MEMORY_CHECK;
        {
            DEBUG_OUT(2,"ins_doub...");
            ptr_t ins_doub = DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward_ptr_t());
            DEBUG_OUT(2,"check");
            EXPECT_EQ(INVALID,ins_doub);
        }
        MEMORY_CHECK;
        {
            DEBUG_OUT(2,"ins...");
            ptr_t ins = DoublyLinkedInstance::create(action_ptr_t(), observation_ptr_t(), reward(0));
            DEBUG_OUT(2,"check");
            EXPECT_NE(INVALID,ins);
            EXPECT_EQ(INVALID,ins->non_const_next());
            EXPECT_EQ(INVALID,ins->non_const_prev());
        }
        MEMORY_CHECK;
    }

    TEST(InstanceTest, Loop) {
        {
            ptr_t ins0 = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(0));
            ptr_t ins1 = DoublyLinkedInstance::create(action_ptr_t(),observation_ptr_t(),reward(1));
            ins0->set_non_const_predecessor(ins1);
            ins0->set_non_const_successor(ins1);
            ins1->set_non_const_predecessor(ins0);
            ins1->set_non_const_successor(ins0);
            DEBUG_OUT(2,"Forwards loop..");
            // forwards loop
            {
                int counter = 0;
                for(auto ins = ins1->non_const_next(); counter<10; ++counter, ins=ins->non_const_next()) {
                    EXPECT_EQ(counter%2,ins->reward->get_value());
                }
            }
            DEBUG_OUT(2,"Backwards loop..");
            // backwards loop
            {
                int counter = 0;
                for(auto ins = ins1->non_const_prev(); counter<10; ++counter, ins=ins->non_const_prev()) {
                    EXPECT_EQ(counter%2,ins->reward->get_value());
                }
            }
            DEBUG_OUT(2,"Detachment..");
            EXPECT_EQ(2,ins0->detach_reachable());
        }
        MEMORY_CHECK;
    }

    TEST(InstanceTest, Iteration) {
        {
            // create
            ptr_t ins3 = create_test_sequence();
            ptr_t ins8 = ins3->non_const_next();
            // negative argument
            EXPECT_EQ(ins8->non_const_prev(2),ins8->non_const_next(-2));
            EXPECT_EQ(ins8->non_const_next(2),ins8->non_const_prev(-2));
            EXPECT_EQ(ins8->const_prev(2),ins8->const_next(-2));
            EXPECT_EQ(ins8->const_next(2),ins8->const_prev(-2));
            // non-const iteration
            EXPECT_EQ(3,ins3->reward->get_value());
            EXPECT_EQ(0,ins3->non_const_prev(3)->reward->get_value());
            EXPECT_EQ(8,ins3->non_const_next(1)->reward->get_value());
            EXPECT_EQ(10,ins3->non_const_next(3)->reward->get_value());
            EXPECT_EQ(4,ins3->non_const_next(1)->non_const_prev(4)->reward->get_value());
            EXPECT_EQ(10,ins3->non_const_next(1)->non_const_prev(4)->non_const_next(6)->reward->get_value());
            EXPECT_EQ(0,ins3->non_const_first()->reward->get_value());
            EXPECT_EQ(10,ins3->non_const_last()->reward->get_value());
            EXPECT_EQ(4,ins3->non_const_last()->non_const_first()->reward->get_value());
            // reconnect
            ins3->set_const_successor(ins8);
            // non-const iteration with const link
            EXPECT_EQ(0,ins3->non_const_prev(3)->reward->get_value());
            EXPECT_EQ(INVALID,ins3->non_const_next(1));
            EXPECT_EQ(10,ins8->non_const_next(2)->reward->get_value());
            EXPECT_EQ(4,ins8->non_const_prev(4)->reward->get_value());
            EXPECT_EQ(0,ins3->non_const_first()->reward->get_value());
            EXPECT_EQ(3,ins3->non_const_last()->reward->get_value());
            EXPECT_EQ(4,ins8->non_const_first()->reward->get_value());
            EXPECT_EQ(10,ins8->non_const_last()->reward->get_value());
            // const iteration
            EXPECT_EQ(0,ins3->const_prev(3)->reward->get_value());
            EXPECT_EQ(8,ins3->const_next()->reward->get_value());
            EXPECT_EQ(10,ins8->const_next(2)->reward->get_value());
            EXPECT_EQ(4,ins8->const_prev(4)->reward->get_value());
            EXPECT_EQ(0,ins3->const_first()->reward->get_value());
            EXPECT_EQ(10,ins3->const_last()->reward->get_value());
            EXPECT_EQ(4,ins8->const_first()->reward->get_value());
            EXPECT_EQ(10,ins8->const_last()->reward->get_value());
            // detach
            EXPECT_EQ(4,ins3->detach_reachable());
            EXPECT_EQ(7,ins8->detach_reachable());
        }
        MEMORY_CHECK;
    }

    TEST(InstanceTest, Detach) {
        {
            set<shared_ptr_t> ins_set;

            // create and detach nr 6 (should really detach only nr 6)
            EXPECT_EQ(1,create_test_sequence(&ins_set)->non_const_next()->non_const_prev(2)->detach());

            // check use counts
            for(shared_ptr_t i : ins_set) {
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
                switch((int)i->reward->get_value()) {
                case 6:
                    EXPECT_EQ(2,i.use_count());
                    break;
                case 8:
                    EXPECT_EQ(8,i.use_count());
                    break;
                case 1:
                case 2:
                case 9:
                    EXPECT_EQ(6,i.use_count());
                    break;
                default:
                    EXPECT_EQ(4,i.use_count());
                }
            }

            // detach rest
            for(shared_ptr_t i : ins_set) {
                // result depends on order, therefore no check
                i->detach();
            }
        }
        MEMORY_CHECK;
    }

    TEST(InstanceTest, DetachReachable) {
        {
            set<shared_ptr_t> ins_set;

            // create and detach reachable from nr 3 (should detach all other)
            EXPECT_EQ(11,create_test_sequence(&ins_set)->detach_reachable());

            // check use counts
            for(shared_ptr_t i : ins_set) {
                EXPECT_EQ(2,i.use_count());
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
            }

            // detach rest
            for(shared_ptr_t i : ins_set) {
                // result depends on order, therefore no check
                i->detach();
            }
        }
        MEMORY_CHECK;
        {
            set<shared_ptr_t> ins_set;

            // create and detach reachable from nr 6 (should detach 4 to 10 but not 0 to 3)
            EXPECT_EQ(7,create_test_sequence(&ins_set)->non_const_next()->non_const_prev(2)->detach_reachable());

            // check use counts
            for(shared_ptr_t i : ins_set) {
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
                switch((int)i->reward->get_value()) {
                case 3:
                case 0:
                    EXPECT_EQ(4,i.use_count());
                    break;
                case 1:
                case 2:
                    EXPECT_EQ(6,i.use_count());
                    break;
                default:
                    EXPECT_EQ(2,i.use_count());
                }
            }

            // detach rest
            for(shared_ptr_t i : ins_set) {
                // result depends on order, therefore no check
                i->detach();
            }
        }
        MEMORY_CHECK;
    }

    TEST(InstanceTest, DetachAll) {
        {
            set<shared_ptr_t> ins_set;

            // create and detach all from nr 6 (should detach all by also going inversely to nr 3 etc.)
            EXPECT_EQ(11,create_test_sequence(&ins_set)->non_const_next()->non_const_prev(2)->detach_all());

            // check use counts and iteration
            for(shared_ptr_t i : ins_set) {
                EXPECT_EQ(2,i.use_count());
                EXPECT_EQ(INVALID,i->non_const_prev());
                EXPECT_EQ(INVALID,i->non_const_next());
            }
        }
        MEMORY_CHECK;
    }

    TEST(InstanceTest, Const) {
        {
            set<shared_ptr_t> ins_set;

            {
                // get sequence and reconnect 3 to 8 via const link
                DEBUG_OUT(2,"Create sequence...");
                ptr_t ins3 = create_test_sequence(&ins_set);
                const_ptr_t ins8 = ins3->const_next();

                // check use_count of 8 before and after reconnection (should
                // have one use count less, since 3 only hold a const ptr but no
                // non-const ptr)
                DEBUG_OUT(2,"Reconnect...");
                EXPECT_EQ(9,((shared_const_ptr_t)ins8).use_count());
                ins3->set_const_successor(ins8);
                EXPECT_EQ(8,((shared_const_ptr_t)ins8).use_count());

                // check iteration
                DEBUG_OUT(2,"Check iteration...");
                EXPECT_EQ(INVALID,ins3->non_const_next());
                EXPECT_EQ(8,ins3->const_next()->reward->get_value());

                // check detach_reachable (should detach only 0 to 3, not all as above)
                DEBUG_OUT(2,"Detach...");
                EXPECT_EQ(4,ins3->detach_reachable());
            }

            // check use counts
            DEBUG_OUT(2,"Check use counts...");
            for(shared_ptr_t i : ins_set) {
                DEBUG_OUT(1, *i << " has " << i.use_count() << " owners");
                switch((int)i->reward->get_value()) {
                case 0:
                case 1:
                case 2:
                case 3:
                    EXPECT_EQ(2,i.use_count());
                    break;
                case 4:
                case 10:
                    EXPECT_EQ(4,i.use_count());
                    break;
                default:
                    EXPECT_EQ(6,i.use_count());
                }
            }

            // detach rest
            for(shared_ptr_t i : ins_set) {
                // result depends on order, therefore no check
                i->detach();
            }


        }
        MEMORY_CHECK;
    }

}; // end namespace
