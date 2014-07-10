#include <gtest/gtest.h>

#include "../Config.h"

#include "../util/util.h"
#include "../util/ProgressBar.h"

#include "../Representation/Feature.h"
#include "../Representation/AbstractAction.h"
#include "MinimalEnvironmentExample/MinimalAction.h"
#include "../Maze/MazeAction.h"
#include "../Maze/AugmentedMazeAction.h"
#include "../Representation/AbstractObservation.h"
#include "MinimalEnvironmentExample/MinimalObservation.h"
#include "../Maze/MazeObservation.h"
#include "../Representation/AbstractReward.h"
#include "MinimalEnvironmentExample/MinimalReward.h"
#include "../Representation/ListedReward.h"
#include "../Representation/AbstractInstance.h"
#include "../Representation/DoublyLinkedInstance.h"

#include "RandomElements.h"

#include <vector>
#include <set>
#include <list>
#include <memory> // shared_ptr
#include <sstream>

#define DEBUG_LEVEL 2
#include "../util/debug.h"

using std::vector;
using std::list;
using std::set;
using std::string;
using std::stringstream;
using std::shared_ptr;
using std::dynamic_pointer_cast;
using util::random_select;

USE_CONFIG_TYPEDEFS;

int get_time_delay();
f_ptr_t get_basis_feature();
f_ptr_t get_const_feature();
f_ptr_t get_action_feature();
f_ptr_t get_observation_feature();
f_ptr_t get_reward_feature();
f_ptr_t get_and_feature();

TEST(FeatureTest, Evaluation) {

    /** Purpose: Check if action, observation, reward, and AND features evaluate
     * correcly. The MinimalEnvironment is used as test bed. This test should
     * also check the different versions of the evaluate method
     * [evaluate(instance), evaluate(instance->const_prev(), instance->action,
     * instance->observation, instance->reward), evaluate(look_up_map_t)]. */

    {
        // representation
        action_ptr_t stay(new MinimalAction(MinimalAction::STAY));
        action_ptr_t change(new MinimalAction(MinimalAction::CHANGE));
        observation_ptr_t red(new MinimalObservation(MinimalObservation::RED));
        observation_ptr_t green(new MinimalObservation(MinimalObservation::GREEN));
        reward_ptr_t some_reward(new MinimalReward(MinimalReward::SOME_REWARD));
        reward_ptr_t no_reward(new MinimalReward(MinimalReward::NO_REWARD));

        // create a specific features
        f_ptr_t action_f = ActionFeature::create(change,-2);
        f_ptr_t observation_f = ObservationFeature::create(green,-2);
        f_ptr_t reward_f = RewardFeature::create(some_reward,-2);
        f_ptr_t and_f_ao(new AndFeature(action_f,observation_f));
        f_ptr_t and_f_or(new AndFeature(observation_f,reward_f));
        f_ptr_t and_f_ar(new AndFeature(action_f,reward_f));
        f_ptr_t and_all(new AndFeature(and_f_ao,and_f_or,and_f_ar));

        // create a sequence with all possible triplets
        instance_ptr_t instance = DoublyLinkedInstance::create(stay,red,some_reward);
        instance = instance->append(stay,red,no_reward);
        instance = instance->append(stay,green,some_reward);
        instance = instance->append(stay,green,no_reward);
        instance = instance->append(change,red,some_reward);
        instance = instance->append(change,red,no_reward);
        instance = instance->append(change,green,some_reward);
        instance = instance->append(change,green,no_reward);

        // create basis feature maps
        typedef Feature::look_up_map_t map_t;
        vector<map_t> bf_maps;
        for(instance_ptr_t ins = instance->non_const_first(); ins!=util::INVALID; ++ins) {
            bf_maps.push_back(map_t());
            map_t& map = bf_maps.back();
            map.insert_feature(action_f,action_f->evaluate(ins));
            map.insert_feature(observation_f,observation_f->evaluate(ins));
            map.insert_feature(reward_f,reward_f->evaluate(ins));
        }

        DEBUG_OUT(2,"Sequence:");
        for(instance_ptr_t i = instance->non_const_first(); i!=util::INVALID; ++i) {
            DEBUG_OUT(2,i);
        }

        DEBUG_OUT(2,"A	O	R	A+O	O+R	A+R	A+O+R	I");
        int ins_counter = 0;
        for(instance_ptr_t ins = instance->non_const_first(); ins!=util::INVALID; ++ins, ++ins_counter) {
            const map_t& map = bf_maps[ins_counter];
            DEBUG_OUT(2,action_f->evaluate(ins) << "/" << action_f->evaluate(ins,stay,red,some_reward) << "/" << action_f->evaluate(map) << "	" <<
                      observation_f->evaluate(ins) << "/" << observation_f->evaluate(ins,stay,red,some_reward) << "/" << observation_f->evaluate(map) << "	" <<
                      reward_f->evaluate(ins) << "/" << reward_f->evaluate(ins,stay,red,some_reward) << "/" << reward_f->evaluate(map) << "	" <<
                      and_f_ao->evaluate(ins) << "/" << and_f_ao->evaluate(ins,stay,red,some_reward) << "/" << and_f_ao->evaluate(map) << "	" <<
                      and_f_or->evaluate(ins) << "/" << and_f_or->evaluate(ins,stay,red,some_reward) << "/" << and_f_or->evaluate(map) << "	" <<
                      and_f_ar->evaluate(ins) << "/" << and_f_ar->evaluate(ins,stay,red,some_reward) << "/" << and_f_ar->evaluate(map) << "	" <<
                      and_all->evaluate(ins) << "/" << and_all->evaluate(ins,stay,red,some_reward) << "/" << and_all->evaluate(map) << "	" <<
                      ins
                );
            switch(ins_counter) {
            case 0:
                EXPECT_EQ(0,      action_f->evaluate(ins));
                EXPECT_EQ(0, observation_f->evaluate(ins));
                EXPECT_EQ(0,      reward_f->evaluate(ins));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins));
                EXPECT_EQ(0,      and_f_or->evaluate(ins));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins));
                EXPECT_EQ(0,       and_all->evaluate(ins));
                EXPECT_EQ(0,      action_f->evaluate(map));
                EXPECT_EQ(0, observation_f->evaluate(map));
                EXPECT_EQ(0,      reward_f->evaluate(map));
                EXPECT_EQ(0,      and_f_ao->evaluate(map));
                EXPECT_EQ(0,      and_f_or->evaluate(map));
                EXPECT_EQ(0,      and_f_ar->evaluate(map));
                EXPECT_EQ(0,       and_all->evaluate(map));
                EXPECT_EQ(0,      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0, observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 1:
                EXPECT_EQ(0,      action_f->evaluate(ins));
                EXPECT_EQ(0, observation_f->evaluate(ins));
                EXPECT_EQ(0,      reward_f->evaluate(ins));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins));
                EXPECT_EQ(0,      and_f_or->evaluate(ins));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins));
                EXPECT_EQ(0,       and_all->evaluate(map));
                EXPECT_EQ(0,      action_f->evaluate(map));
                EXPECT_EQ(0, observation_f->evaluate(map));
                EXPECT_EQ(0,      reward_f->evaluate(map));
                EXPECT_EQ(0,      and_f_ao->evaluate(map));
                EXPECT_EQ(0,      and_f_or->evaluate(map));
                EXPECT_EQ(0,      and_f_ar->evaluate(map));
                EXPECT_EQ(0,       and_all->evaluate(ins));
                EXPECT_EQ(0,      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0, observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 2:
                EXPECT_EQ(0,      action_f->evaluate(ins));
                EXPECT_EQ(0, observation_f->evaluate(ins));
                EXPECT_EQ(1,      reward_f->evaluate(ins));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins));
                EXPECT_EQ(0,      and_f_or->evaluate(ins));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins));
                EXPECT_EQ(0,       and_all->evaluate(ins));
                EXPECT_EQ(0,      action_f->evaluate(map));
                EXPECT_EQ(0, observation_f->evaluate(map));
                EXPECT_EQ(1,      reward_f->evaluate(map));
                EXPECT_EQ(0,      and_f_ao->evaluate(map));
                EXPECT_EQ(0,      and_f_or->evaluate(map));
                EXPECT_EQ(0,      and_f_ar->evaluate(map));
                EXPECT_EQ(0,       and_all->evaluate(map));
                EXPECT_EQ(0,      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0, observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 3:
                EXPECT_EQ(0,      action_f->evaluate(ins));
                EXPECT_EQ(0, observation_f->evaluate(ins));
                EXPECT_EQ(0,      reward_f->evaluate(ins));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins));
                EXPECT_EQ(0,      and_f_or->evaluate(ins));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins));
                EXPECT_EQ(0,       and_all->evaluate(ins));
                EXPECT_EQ(0,      action_f->evaluate(map));
                EXPECT_EQ(0, observation_f->evaluate(map));
                EXPECT_EQ(0,      reward_f->evaluate(map));
                EXPECT_EQ(0,      and_f_ao->evaluate(map));
                EXPECT_EQ(0,      and_f_or->evaluate(map));
                EXPECT_EQ(0,      and_f_ar->evaluate(map));
                EXPECT_EQ(0,       and_all->evaluate(map));
                EXPECT_EQ(0,      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1, observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 4:
                EXPECT_EQ(0,      action_f->evaluate(ins));
                EXPECT_EQ(1, observation_f->evaluate(ins));
                EXPECT_EQ(1,      reward_f->evaluate(ins));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins));
                EXPECT_EQ(1,      and_f_or->evaluate(ins));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins));
                EXPECT_EQ(0,       and_all->evaluate(ins));
                EXPECT_EQ(0,      action_f->evaluate(map));
                EXPECT_EQ(1, observation_f->evaluate(map));
                EXPECT_EQ(1,      reward_f->evaluate(map));
                EXPECT_EQ(0,      and_f_ao->evaluate(map));
                EXPECT_EQ(1,      and_f_or->evaluate(map));
                EXPECT_EQ(0,      and_f_ar->evaluate(map));
                EXPECT_EQ(0,       and_all->evaluate(map));
                EXPECT_EQ(0,      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1, observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 5:
                EXPECT_EQ(0,      action_f->evaluate(ins));
                EXPECT_EQ(1, observation_f->evaluate(ins));
                EXPECT_EQ(0,      reward_f->evaluate(ins));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins));
                EXPECT_EQ(0,      and_f_or->evaluate(ins));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins));
                EXPECT_EQ(0,       and_all->evaluate(ins));
                EXPECT_EQ(0,      action_f->evaluate(map));
                EXPECT_EQ(1, observation_f->evaluate(map));
                EXPECT_EQ(0,      reward_f->evaluate(map));
                EXPECT_EQ(0,      and_f_ao->evaluate(map));
                EXPECT_EQ(0,      and_f_or->evaluate(map));
                EXPECT_EQ(0,      and_f_ar->evaluate(map));
                EXPECT_EQ(0,       and_all->evaluate(map));
                EXPECT_EQ(1,      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0, observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 6:
                EXPECT_EQ(1,      action_f->evaluate(ins));
                EXPECT_EQ(0, observation_f->evaluate(ins));
                EXPECT_EQ(1,      reward_f->evaluate(ins));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins));
                EXPECT_EQ(0,      and_f_or->evaluate(ins));
                EXPECT_EQ(1,      and_f_ar->evaluate(ins));
                EXPECT_EQ(0,       and_all->evaluate(ins));
                EXPECT_EQ(1,      action_f->evaluate(map));
                EXPECT_EQ(0, observation_f->evaluate(map));
                EXPECT_EQ(1,      reward_f->evaluate(map));
                EXPECT_EQ(0,      and_f_ao->evaluate(map));
                EXPECT_EQ(0,      and_f_or->evaluate(map));
                EXPECT_EQ(1,      and_f_ar->evaluate(map));
                EXPECT_EQ(0,       and_all->evaluate(map));
                EXPECT_EQ(1,      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0, observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(0,       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 7:
                EXPECT_EQ(1,      action_f->evaluate(ins));
                EXPECT_EQ(0, observation_f->evaluate(ins));
                EXPECT_EQ(0,      reward_f->evaluate(ins));
                EXPECT_EQ(0,      and_f_ao->evaluate(ins));
                EXPECT_EQ(0,      and_f_or->evaluate(ins));
                EXPECT_EQ(0,      and_f_ar->evaluate(ins));
                EXPECT_EQ(0,       and_all->evaluate(ins));
                EXPECT_EQ(1,      action_f->evaluate(map));
                EXPECT_EQ(0, observation_f->evaluate(map));
                EXPECT_EQ(0,      reward_f->evaluate(map));
                EXPECT_EQ(0,      and_f_ao->evaluate(map));
                EXPECT_EQ(0,      and_f_or->evaluate(map));
                EXPECT_EQ(0,      and_f_ar->evaluate(map));
                EXPECT_EQ(0,       and_all->evaluate(map));
                EXPECT_EQ(1,      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1, observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(1,       and_all->evaluate(ins,stay,red,some_reward));
                break;
            default:
                DEBUG_DEAD_LINE;
                EXPECT_TRUE(false);
            }
        }

        instance->detach_reachable();
    }
    EXPECT_TRUE(AbstractInstance::empty_memory_check());
}

TEST(FeatureTest, SharedPtr) {

    /** Purpose: AndFeature objects hold shared pointers to their basis features
     * in a "flat" set, i.e., an AndFeature 'A' constructed using another
     * AndFeature 'B' should NOT hold a pointer to B but instead merge B's set
     * of basis features into its own set. This test should check this behaviour
     * by checking shared pointer use counts. */

    // vector holding pointers to basis features
    vector<f_ptr_t> basis_features;

    // Construct basis features
    DEBUG_OUT(2,"Construct basis features:")
    basis_features.push_back(ConstFeature::create(3.5));
    DEBUG_OUT(2,"    " << *basis_features.back())
    basis_features.push_back(ActionFeature::create(action_ptr_t(new MinimalAction()), 0));
    DEBUG_OUT(2,"    " << *basis_features.back())
    basis_features.push_back(ObservationFeature::create(observation_ptr_t(new MinimalObservation()), 0));
    DEBUG_OUT(2,"    " << *basis_features.back())
    basis_features.push_back(RewardFeature::create(reward_ptr_t(new MinimalReward()), 0));
    DEBUG_OUT(2,"    " << *basis_features.back())

    // Test counters of shared pointers
    for(auto b : basis_features) {
        // one in basis_features and one in loop variable
        EXPECT_EQ(b.use_count(),2);
    }

    // Construct compound features
    f_ptr_t a1(new AndFeature(basis_features[0]));
    f_ptr_t a2(new AndFeature(basis_features[1],basis_features[2]));
    f_ptr_t a3(new AndFeature(a1,basis_features[3]));

    // Re-test counters
    EXPECT_EQ(3, basis_features[0].use_count()) << "in vector and two AND features (a1,a3)";
    EXPECT_EQ(2, basis_features[1].use_count()) << "in vector and one AND feature (a2)";
    EXPECT_EQ(2, basis_features[2].use_count()) << "in vector and one AND feature(a2)";
    EXPECT_EQ(2, basis_features[3].use_count()) << "in vector and one AND feature(a3)";
    EXPECT_EQ(1, a1.use_count()) << "only a1 itself";
    EXPECT_EQ(1, a2.use_count()) << "only a2 itself";
    EXPECT_EQ(1, a3.use_count()) << "only a3 itself";
}

TEST(FeatureTest, ComparisonAndOrdering) {

    /** Purpose: Some basic tests for comparison and ordering of features. A set
     * of Test features is generated randomly.
     *
     * "True" equality is defined by comparing string description (ListedReward
     * and MazeObservation have to be compiled with DEBUG_LEVEL >= 2 to
     * correctly differentiate between, e.g., (1,1) position in a 3x3 maze and a
     * 5x5 maze). Equality operator is checked against "true" equality (via
     * string description).
     *
     * Inequality should be the negation of equality.
     *
     * For BasisFeature pointers to (semantically) identical objects should
     * actually point to the same object.
     *
     * Ordering is checked as follows: If two features 'A' and 'B' are equal
     * neither A<B nor B<A should hold true; if A and B are not equal either A<B
     * or B<A should hold true.
     *
     * Additionally a sorted std::list of features is constructed by
     * consecutively inserting features (details below) and sorting is checked.
     *
     * f_set_t and f_ptr_set_t is compared against the sorted list. We expect
     * (1) f_ptr_set_t to contain a different number of features as the sorted
     * list in a possibly different order since addresses are compared but
     * identical BasisFeature objects are shared (same address) (2) f_set_t to
     * contain a different number of features but in the same order (when
     * ignoring duplicates in the list) since the actual objects are
     * compared. */

    // vector holding pointers to features
    vector<f_ptr_t> feature_vector;

    // Construct a random set of features
    int number_of_features = 500;
    repeat(number_of_features) {
        if(drand48()<0.8) {
            // use basis feature
            feature_vector.push_back(get_basis_feature());
        } else {
            // use 'and' feature
            feature_vector.push_back(get_and_feature());
        }
    }
    DEBUG_OUT(1,"Created " << number_of_features << " random features");


    // check debug level for equality check
    {
        ListedReward r1({0,1},0), r2({0,2},0);
        stringstream s1, s2;
        s1 << r1;
        s2 << r2;
        if(s1.str()==s2.str()) {
            EXPECT_EQ(r1,r2) << "'" << r1 << "' and '" << r2 << "' are different!";
            DEBUG_WARNING("ListedReward has to be compiled with (at least) DEBUG_LEVEL 2 for equality check to work properly!");
        }
    }
    {
        MazeObservation o1(1,1,0,0), o2(2,2,0,0);
        stringstream s1, s2;
        s1 << o1;
        s2 << o2;
        if(s1.str()==s2.str()) {
            EXPECT_EQ(o1,o2) << "'" << o1 << "' and '" << o2 << "' are different!";
            DEBUG_WARNING("MazeObservation has to be compiled with (at least) DEBUG_LEVEL 2 for equality check to work properly!");
        }
    }

    // check equality, inequality, and ordering via description
    {
        int counter = 0;
        int ptr_equal_check_counter = 0;
        ProgressBar::init("Checking Pairwise Operators: ");
        for(f_ptr_t f1 : feature_vector) {
            for(f_ptr_t f2 : feature_vector) {
                // check equality
                stringstream s1, s2;
                s1 << *f1;
                s2 << *f2;
                bool description_equal = s1.str()==s2.str();
                if(*f1==*f2) {
                    EXPECT_TRUE(description_equal) << "'" << *f1 << "' == '" << *f2 << "'";
                    // expect equal BasisFeature objects to be created only once
                    // and share ownership
                    auto f1_basis = dynamic_pointer_cast<const BasisFeature>(f1);
                    auto f2_basis = dynamic_pointer_cast<const BasisFeature>(f2);
                    // should be both nullptr or point to the same object
                    EXPECT_EQ(f1_basis,f2_basis)
                        << "    (" << f1_basis << ") <--> " << *f1 << "\n"
                        << "    (" << f2_basis << ") <--> " << *f2;
                    ++ptr_equal_check_counter;
                } else {
                    EXPECT_FALSE(description_equal) << "'" << *f1 << "' != '" << *f2 << "'";
                }
                // check inequality
                if(*f1==*f2) {
                    EXPECT_FALSE(*f1!=*f2) << "inequality is not the negation of equality";
                } else {
                    EXPECT_TRUE(*f1!=*f2) << "inequality is not the negation of equality";
                }
                // check ordering
                if(*f1==*f2) {
                    EXPECT_FALSE(*f1<*f2) << *f1 << "==" << *f2 << " violated by ordering operator";
                    EXPECT_FALSE(*f2<*f1) << *f1 << "==" << *f2 << " violated by ordering operator";
                } else {
                    EXPECT_TRUE((*f1<*f2 || *f2<*f1) && !(*f1<*f2 && *f2<*f1)) << *f1 << "!=" << *f2 << " violated by ordering operator";
                }
            }
            ProgressBar::print(counter++, number_of_features);
        }
        ProgressBar::terminate();
        DEBUG_OUT(1,"Performed " << ptr_equal_check_counter << " object-pointer equality checks");
    }

    // check ordering
    {
        // do stupid sorting
        int counter = 0;
        list<f_ptr_t> feature_list;
        ProgressBar::init("Checking Ordering: ");
        for(f_ptr_t f_unsorted : feature_vector) {
            // f_unsorted is inserted into feature_list right before
            // insert_before (first element that is not
            // smaller). some_elem_after should should never compare less to
            // f_unsorted for the rest of feature_list.
            auto insert_before = feature_list.begin();   // first element that is not smaller
            auto some_elem_after = feature_list.begin(); // all other elements after that
            bool found = false;
            while(insert_before!=feature_list.end() && some_elem_after!=feature_list.end()) {
                if(!found && **insert_before<*f_unsorted) {
                    ++insert_before;
                } else {
                    if(!found) {
                        found = true;
                        some_elem_after = insert_before;
                    }
                    ++some_elem_after;
                    if(some_elem_after!=feature_list.end()) {
                        EXPECT_FALSE(**some_elem_after<*f_unsorted) << **some_elem_after << " should be larger or equal to " << *f_unsorted;
                    }
                }
            }
            feature_list.insert(insert_before,f_unsorted);
            ProgressBar::print(counter++, number_of_features);
        }
        ProgressBar::terminate();

        // check sorting again by pairwise comparison
        {
            for(auto low_elem=feature_list.begin(); low_elem!=feature_list.end(); ++low_elem) {
                for(auto high_elem=low_elem; high_elem!=feature_list.end(); ++high_elem) {
                    EXPECT_FALSE(**high_elem<**low_elem) << **high_elem << "<" << **low_elem;
                }
            }
        }

        // compare/check against f_ptr_set_t and f_set_t
        {
            // construct feature set
            f_set_t f_set;
            f_ptr_set_t f_ptr_set;
            for(f_ptr_t feature : feature_vector) {
                f_set.insert(feature);
                f_ptr_set.insert(feature);
            }

            // iterate through f_set_t and list
            auto list_it = feature_list.begin();
            auto set_it = f_set.begin();
            while(list_it!=feature_list.end() && set_it!=f_set.end()) {
                // compare
                EXPECT_EQ(**list_it,**set_it);
                DEBUG_OUT(3,"");
                DEBUG_OUT(3,**list_it);
                DEBUG_OUT(3,**set_it);
                // skip multiple occurrences in list
                auto next_list_it = list_it;
                ++next_list_it;
                while(next_list_it!=feature_list.end() && **next_list_it==**list_it) {
                    DEBUG_OUT(3,"    Skipping " << **next_list_it << " because of equality");
                    list_it = next_list_it;
                    ++next_list_it;
                }
                // increment
                ++list_it;
                ++set_it;
            }

            // generally feature_list will have a greater size than f_set and
            // f_ptr_set (which also have different size) but the same as the
            // unordered feature_vector
            EXPECT_GT(feature_list.size(),f_set.size());
            EXPECT_GT(feature_list.size(),f_ptr_set.size());
            EXPECT_NE(f_set.size(),f_ptr_set.size());
            EXPECT_EQ(feature_list.size(),feature_vector.size());
        }


    }
}

int get_time_delay() {
    return rand()%11-5;
}

f_ptr_t get_basis_feature() {
    // choose feature type at random
    switch(random_select<Feature::FEATURE_TYPE>({
                    Feature::FEATURE_TYPE::CONST_FEATURE,
                    Feature::FEATURE_TYPE::ACTION,
                    Feature::FEATURE_TYPE::OBSERVATION,
                    Feature::FEATURE_TYPE::REWARD
                    })) {
    case Feature::FEATURE_TYPE::CONST_FEATURE: {
        return get_const_feature();
    }
    case Feature::FEATURE_TYPE::ACTION: {
        return get_action_feature();
    }
    case Feature::FEATURE_TYPE::OBSERVATION: {
        return get_observation_feature();
    }
    case Feature::FEATURE_TYPE::REWARD: {
        return get_reward_feature();
    }
    default:
        DEBUG_ERROR("Unexpected type");
        EXPECT_TRUE(false);
        return f_ptr_t();
    }
}

f_ptr_t get_const_feature() {
    return ConstFeature::create(drand48());
}

f_ptr_t get_action_feature() {
    return ActionFeature::create(get_random_action(),get_time_delay());
}

f_ptr_t get_observation_feature() {
    return ObservationFeature::create(get_random_observation(),get_time_delay());
}

f_ptr_t get_reward_feature() {
    return RewardFeature::create(get_random_reward(),get_time_delay());
}

f_ptr_t get_and_feature() {
    int number_of_subfeatures = rand()%5 + 1;
    f_ptr_t and_feature(get_basis_feature());
    for(int idx=1; idx<number_of_subfeatures; ++idx) {
        and_feature = f_ptr_t(new AndFeature(and_feature,get_basis_feature()));
    }
    return and_feature;
}
