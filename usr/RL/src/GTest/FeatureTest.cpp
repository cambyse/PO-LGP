#include <gtest/gtest.h>

#include <config/Config.h>

#include <util/util.h>
#include <util/ProgressBar.h>

#include <representation/Feature.h>
#include <representation/AbstractAction.h>
#include "MinimalEnvironmentExample/MinimalAction.h"
#include <Maze/MazeAction.h>
#include <ButtonWorld/ButtonAction.h>
#include <Maze/AugmentedMazeAction.h>
#include <representation/AbstractObservation.h>
#include "MinimalEnvironmentExample/MinimalObservation.h"
#include <Maze/MazeObservation.h>
#include <representation/AbstractReward.h>
#include "MinimalEnvironmentExample/MinimalReward.h"
#include <representation/ListedReward.h>
#include <representation/AbstractInstance.h>
#include <representation/DoublyLinkedInstance.h>

#include "RandomElements.h"

#include <vector>
#include <utility>
#include <tuple>
#include <set>
#include <set>
#include <list>
#include <memory> // shared_ptr
#include <sstream>
#include <string>

#define DEBUG_LEVEL 2
#include <util/debug.h>

using std::vector;
using std::list;
using std::set;
using std::string;
using std::pair;
using std::make_pair;
using std::tuple;
using std::make_tuple;
using std::get;
using std::string;
using std::stringstream;
using std::shared_ptr;
using std::dynamic_pointer_cast;
using util::random_select;

USE_CONFIG_TYPEDEFS;

enum class MODE { RANDOM, REUSE, UNIQUE, PRINT };

int get_time_delay();
f_ptr_t get_basis_feature(MODE mode, bool clear = true);
f_ptr_t get_const_feature(MODE mode);
f_ptr_t get_action_feature(MODE mode);
f_ptr_t get_button_action_feature(MODE mode);
f_ptr_t get_observation_feature(MODE mode);
f_ptr_t get_reward_feature(MODE mode);
set<pair<f_ptr_t, string> > get_random_basis_feature_set();
f_ptr_t get_and_feature(MODE mode);

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
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(map));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(map));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(map));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(map));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(map));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(map));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(map));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 1:
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(map));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(map));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(map));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(map));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(map));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(map));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(map));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 2:
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(ins));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(map));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(map));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(map));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(map));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(map));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(map));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(map));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 3:
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(map));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(map));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(map));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(map));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(map));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(map));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(map));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ( observation_f->return_function(1), observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_or->return_function(1),      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 4:
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins));
                EXPECT_EQ( observation_f->return_function(1), observation_f->evaluate(ins));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(ins));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins));
                EXPECT_EQ(      and_f_or->return_function(1),      and_f_or->evaluate(ins));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(map));
                EXPECT_EQ( observation_f->return_function(1), observation_f->evaluate(map));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(map));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(map));
                EXPECT_EQ(      and_f_or->return_function(1),      and_f_or->evaluate(map));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(map));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(map));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ( observation_f->return_function(1), observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 5:
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(ins));
                EXPECT_EQ( observation_f->return_function(1), observation_f->evaluate(ins));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins));
                EXPECT_EQ(      action_f->return_function(0),      action_f->evaluate(map));
                EXPECT_EQ( observation_f->return_function(1), observation_f->evaluate(map));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(map));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(map));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(map));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(map));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(map));
                EXPECT_EQ(      action_f->return_function(1),      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ar->return_function(1),      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 6:
                EXPECT_EQ(      action_f->return_function(1),      action_f->evaluate(ins));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(ins));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins));
                EXPECT_EQ(      and_f_ar->return_function(1),      and_f_ar->evaluate(ins));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins));
                EXPECT_EQ(      action_f->return_function(1),      action_f->evaluate(map));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(map));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(map));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(map));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(map));
                EXPECT_EQ(      and_f_ar->return_function(1),      and_f_ar->evaluate(map));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(map));
                EXPECT_EQ(      action_f->return_function(1),      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins,stay,red,some_reward));
                break;
            case 7:
                EXPECT_EQ(      action_f->return_function(1),      action_f->evaluate(ins));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(ins));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(ins));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(ins));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(ins));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(ins));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(ins));
                EXPECT_EQ(      action_f->return_function(1),      action_f->evaluate(map));
                EXPECT_EQ( observation_f->return_function(0), observation_f->evaluate(map));
                EXPECT_EQ(      reward_f->return_function(0),      reward_f->evaluate(map));
                EXPECT_EQ(      and_f_ao->return_function(0),      and_f_ao->evaluate(map));
                EXPECT_EQ(      and_f_or->return_function(0),      and_f_or->evaluate(map));
                EXPECT_EQ(      and_f_ar->return_function(0),      and_f_ar->evaluate(map));
                EXPECT_EQ(       and_all->return_function(0),       and_all->evaluate(map));
                EXPECT_EQ(      action_f->return_function(1),      action_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ( observation_f->return_function(1), observation_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      reward_f->return_function(1),      reward_f->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ao->return_function(1),      and_f_ao->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_or->return_function(1),      and_f_or->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(      and_f_ar->return_function(1),      and_f_ar->evaluate(ins,stay,red,some_reward));
                EXPECT_EQ(       and_all->return_function(1),       and_all->evaluate(ins,stay,red,some_reward));
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

TEST(FeatureTest, Contradiction) {
    #warning TODO
}

TEST(FeatureTest, RandomAndUniqueFeatures) {
    // Check if identical BasisFeatures are actually identical in memory but
    // identical AndFeatures are not AND f_set_t and f_ptr_set_t treat them
    // correspondingly.
    //
    // f_set_t should not store duplicates at all while f_ptr_set_t should store
    // duplicate AndFeatures but not duplicate BasisFeatures

    int number_of_features = 1000;

    // random basis features
    {
        // the sets
        f_set_t f_set;
        f_ptr_set_t f_ptr_set;

        // construct and check basis features
        repeat(number_of_features) {
            // use basis features only
            f_ptr_t f = get_basis_feature(MODE::RANDOM); // new feature
            f_ptr_t f_copy = get_basis_feature(MODE::REUSE); // identical feature
            EXPECT_EQ(f, f_copy);
            f_set.insert(f);
            f_set.insert(f_copy);
            f_ptr_set.insert(f);
            f_ptr_set.insert(f_copy);
        }
        // intentional (and random) duplicates are not stored because objects
        // compare equal
        EXPECT_GE(number_of_features, f_set.size());
        // intentional (and random) duplicates are not stored because
        // object-addresses are equal
        EXPECT_GE(number_of_features, f_ptr_set.size());
        // identical objects have identical addresses thus sizes must be equal,
        // too
        EXPECT_EQ(f_ptr_set.size(), f_set.size());

        // print
        DEBUG_OUT(1,"Checked " << number_of_features << " pairs of random basis features");
    }

    // unique basis features (same as above but no random duplicates)
    {
        // the sets
        f_set_t f_set;
        f_ptr_set_t f_ptr_set;

        // construct and check basis features
        repeat(number_of_features) {
            // use basis features only
            f_ptr_t f = get_basis_feature(MODE::UNIQUE); // new feature
            f_ptr_t f_copy = get_basis_feature(MODE::REUSE); // identical feature
            EXPECT_EQ(f, f_copy);
            if(f_set.find(f)!=f_set.end()) {
                DEBUG_ERROR(*f << " found in f_set: " << **(f_set.find(f)));
                get_basis_feature(MODE::PRINT);
                EXPECT_TRUE(false) << "Feature OBJECT should be unique";
            }
            if(f_ptr_set.find(f)!=f_ptr_set.end()) {
                DEBUG_ERROR(*f << " found in f_ptr_set: " << **(f_ptr_set.find(f)));
                get_basis_feature(MODE::PRINT);
                EXPECT_TRUE(false) << "Feature ADDRESS should be unique";
            }
            f_set.insert(f);
            f_set.insert(f_copy);
            f_ptr_set.insert(f);
            f_ptr_set.insert(f_copy);
        }
        // intentional duplicates are not stored because objects compare equal
        EXPECT_EQ(number_of_features, f_set.size());
        // intentional duplicates are not stored because object-addresses are
        // equal
        EXPECT_EQ(number_of_features, f_ptr_set.size());
        // identical objects have identical addresses thus sizes must be equal,
        // too
        EXPECT_EQ(f_ptr_set.size(), f_set.size());

        // print
        DEBUG_OUT(1,"Checked " << number_of_features << " pairs of unique basis features");
    }

    // random AND features
    {
        // the sets
        f_set_t f_set;
        f_ptr_set_t f_ptr_set;

        // construct and check AND features
        repeat(number_of_features) {
            // use and features only
            f_ptr_t f = get_and_feature(MODE::RANDOM); // new feature
            f_ptr_t f_copy = get_and_feature(MODE::REUSE); // identical feature
            EXPECT_NE(f, f_copy);
            f_set.insert(f);
            f_set.insert(f_copy);
            f_ptr_set.insert(f);
            f_ptr_set.insert(f_copy);
        }
        // intentional (and random) duplicates are not stored because objects
        // compare equal
        EXPECT_GE(number_of_features, f_set.size());
        // intentional (and random) duplicates ARE stored because
        // object-addresses are NOT equal
        EXPECT_EQ(2*number_of_features, f_ptr_set.size());
        // identical objects do have NON-identical addresses, thus sizes must be
        // different
        EXPECT_GE(f_ptr_set.size(), 2*f_set.size());

        // print
        DEBUG_OUT(1,"Checked " << number_of_features << " pairs of random AND features");
    }

    // unique AND features (same as above but no random duplicates)
    {
        // the sets
        f_set_t f_set;
        f_ptr_set_t f_ptr_set;

        // construct and check AND features
        repeat(number_of_features) {
            // use and features only
            f_ptr_t f = get_and_feature(MODE::UNIQUE); // new feature
            f_ptr_t f_copy = get_and_feature(MODE::REUSE); // identical feature
            EXPECT_NE(f, f_copy);
            if(f_set.find(f)!=f_set.end()) {
                DEBUG_ERROR(*f << " found in f_set: " << **(f_set.find(f)));
                get_and_feature(MODE::PRINT);
                EXPECT_TRUE(false) << "Feature OBJECT should be unique";
            }
            if(f_ptr_set.find(f)!=f_ptr_set.end()) {
                DEBUG_ERROR(*f << " found in f_ptr_set: " << **(f_ptr_set.find(f)));
                get_and_feature(MODE::PRINT);
                EXPECT_TRUE(false) << "Feature ADDRESS should be unique";
            }
            f_set.insert(f);
            f_set.insert(f_copy);
            f_ptr_set.insert(f);
            f_ptr_set.insert(f_copy);
        }
        // intentional duplicates are not stored because objects compare equal
        EXPECT_EQ(number_of_features, f_set.size());
        // intentional duplicates ARE stored because object-addresses are NOT
        // equal
        EXPECT_EQ(2*number_of_features, f_ptr_set.size());
        // identical objects do have NON-identical addresses, thus sizes must be
        // different
        EXPECT_EQ(f_ptr_set.size(), 2*f_set.size());

        // print
        DEBUG_OUT(1,"Checked " << number_of_features << " pairs of unique AND features");
    }
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
     * Inequality of A and B should imply A<B or B<A.
     *
     * BasisFeature pointers to (semantically) identical objects should actually
     * point to the same object in memory.
     *
     * Ordering is checked as follows: If two features 'A' and 'B' are equal
     * neither A<B nor B<A should hold true; if A and B are not equal either A<B
     * or B<A should hold true.
     *
     * Additionally a sorted std::list of features is constructed by
     * consecutively inserting features (details below) and sorting is checked.
     *
     * f_set_t and f_ptr_set_t is compared against the sorted list. We expect
     * (1) f_ptr_set_t to contain (a) a possibly different number of features
     * than the sorted list in (b) a different order since f_ptr_set_t uses
     * addresses but identical BasisFeature objects are shared (same address)
     * (2) f_set_t to (a) contain a different number of features than the sorted
     * list but (b) in the same order (when ignoring duplicates in the list)
     * since the actual objects are compared. */

    // vector holding pointers to features
    vector<f_ptr_t> feature_vector;

    // Construct a random set of features
    int number_of_features = 500;
    repeat(number_of_features) {
        if(true) {
            // use basis feature
            feature_vector.push_back(get_basis_feature(MODE::RANDOM));
        } else {
            // use 'and' feature
            feature_vector.push_back(get_and_feature(MODE::RANDOM));
        }
        DEBUG_OUT(3, *feature_vector.back());
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
                DEBUG_OUT(3, *f1 << " / " << *f2 << " ==:" << (*f1==*f2?"true":"false") << " !=:" << (*f1!=*f2?"true":"false") << " <:" << (*f1<*f2?"true":"false") << " >:" << (*f2<*f1?"true":"false"));
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
            // f_ptr_set (which may have a different size) but the same as the
            // unordered feature_vector
            EXPECT_GT(feature_list.size(),f_set.size());
            EXPECT_GT(feature_list.size(),f_ptr_set.size());
            EXPECT_EQ(feature_list.size(),feature_vector.size());
        }
    }
}

int get_time_delay() {
    return rand()%11-5;
}

f_ptr_t get_basis_feature(MODE mode, bool clear) {
    // vector of feature types
    vector<Feature::FEATURE_TYPE> feature_types({
                // Feature::FEATURE_TYPE::CONST_FEATURE,
                Feature::FEATURE_TYPE::ACTION,
                Feature::FEATURE_TYPE::OBSERVATION,
                Feature::FEATURE_TYPE::REWARD,
                Feature::FEATURE_TYPE::BUTTON_ACTION
                });
    // initialize at random
    static Feature::FEATURE_TYPE type = random_select<Feature::FEATURE_TYPE>(feature_types);
    // modes
    switch(mode) {
    case MODE::RANDOM:
    case MODE::UNIQUE:
        if(clear) {
            // to clear the unique sets
            get_const_feature(MODE::RANDOM);
            get_action_feature(MODE::RANDOM);
            get_button_action_feature(MODE::RANDOM);
            get_observation_feature(MODE::RANDOM);
            get_reward_feature(MODE::RANDOM);
        }
        // use random type
        type = random_select<Feature::FEATURE_TYPE>(feature_types);
        break;
    case MODE::REUSE:
        // just use the same type as in last call
        break;
    case MODE::PRINT:
        // don't do anything (as REUSE) and pass on
        break;
    default:
        DEBUG_DEAD_LINE;
        EXPECT_TRUE(false);
    }
    // return corresponding type (pass on mode)
    switch(type) {
    case Feature::FEATURE_TYPE::CONST_FEATURE: {
        return get_const_feature(mode);
    }
    case Feature::FEATURE_TYPE::ACTION: {
        return get_action_feature(mode);
    }
    case Feature::FEATURE_TYPE::BUTTON_ACTION: {
        return get_button_action_feature(mode);
    }
    case Feature::FEATURE_TYPE::OBSERVATION: {
        return get_observation_feature(mode);
    }
    case Feature::FEATURE_TYPE::REWARD: {
        return get_reward_feature(mode);
    }
    default:
        DEBUG_ERROR("Unexpected type");
        EXPECT_TRUE(false);
        return f_ptr_t();
    }
}

#define RANDOM_REUSE_UNIQUE(type, value, generator, recursion, printer) \
    static type value = generator;                                      \
    static set<type> unique_set;                                        \
    switch(mode) {                                                      \
    case MODE::RANDOM:                                                  \
        /* random */                                                    \
        value = generator;                                              \
        break;                                                          \
    case MODE::REUSE:                                                   \
        /* just reuse old value */                                      \
        break;                                                          \
    case MODE::UNIQUE:                                                  \
        /* random */                                                    \
        value = generator;                                              \
        /* reject if non-unique */                                      \
        if(unique_set.find(value)==unique_set.end()) {                  \
            /* not found --> accept: remember value */                  \
            unique_set.insert(value);                                   \
        } else {                                                        \
            /* found --> reject: recursive call */                      \
            return recursion;                                           \
        }                                                               \
        break;                                                          \
    case MODE::PRINT:                                                   \
        printer(value);                                                 \
        /* just reuse old value */                                      \
        break;                                                          \
    default:                                                            \
        DEBUG_DEAD_LINE;                                                \
        EXPECT_TRUE(false);                                             \
    }

f_ptr_t get_const_feature(MODE mode) {
    RANDOM_REUSE_UNIQUE(int,
                        i,
                        rand()%1000,
                        get_basis_feature(mode),
                        [](int j) { DEBUG_OUT(0, "Used: " << j); }
                        // get_const_feature(mode)
        );
    return ConstFeature::create(i);
}

f_ptr_t get_action_feature(MODE mode) {
    typedef pair<action_ptr_t, int> type;
    RANDOM_REUSE_UNIQUE(type,
                        action_delay,
                        make_pair(get_random_action(), get_time_delay()),
                        get_basis_feature(mode),
                        [](type p) { DEBUG_OUT(0, "Used: " << p.first << " / " << p.second); }
                        // get_action_feature(mode)
        );
    return ActionFeature::create(action_delay.first, action_delay.second);
}

f_ptr_t get_button_action_feature(MODE mode) {
    typedef tuple<int, int, bool> type;
    RANDOM_REUSE_UNIQUE(type,
                        idx_delay_not,
                        make_tuple((rand()%5+1), get_time_delay(), rand()%2==0),
                        get_basis_feature(mode),
                        [](type p) { DEBUG_OUT(0, "Used: "
                                               << get<0>(p) << " / "
                                               << get<1>(p) << " / "
                                               << get<2>(p) ); }
                        // get_button_action_feature(mode)
        );
    return ButtonActionFeature::create(get<0>(idx_delay_not),
                                       get<1>(idx_delay_not),
                                       get<2>(idx_delay_not));
}

f_ptr_t get_observation_feature(MODE mode) {
    typedef pair<observation_ptr_t, int> type;
    RANDOM_REUSE_UNIQUE(type,
                        observation_delay,
                        make_pair(get_random_observation(), get_time_delay()),
                        get_basis_feature(mode),
                        [](type p) { DEBUG_OUT(0, "Used: " << p.first << " / " << p.second); }
                        // get_observation_feature(mode)
        );
    return ObservationFeature::create(observation_delay.first, observation_delay.second);
}

f_ptr_t get_reward_feature(MODE mode) {
    typedef pair<reward_ptr_t, int> type;
    RANDOM_REUSE_UNIQUE(type,
                        reward_delay,
                        make_pair(get_random_reward(), get_time_delay()),
                        get_basis_feature(mode),
                        [](type p) { DEBUG_OUT(0, "Used: " << p.first << " / " << p.second); }
                        // get_reward_feature(mode)
        );
    return RewardFeature::create(reward_delay.first, reward_delay.second);
}

set<pair<f_ptr_t, string> > get_random_basis_feature_set() {
    set<pair<f_ptr_t, string> > f_set;
    repeat(rand()%5 + 1) {
        // add random basis feature but don't clear their unique sets
        f_ptr_t f = get_basis_feature(MODE::RANDOM, false);
        string s = (string)(*f);
        auto p = make_pair(f,s);
        while(f_set.find(p)!=f_set.end()) {
            f = get_basis_feature(MODE::RANDOM, false);
            s = (string)(*f);
            p = make_pair(f,s);
        }
        f_set.insert(p);
    }
    return f_set;
}

f_ptr_t get_and_feature(MODE mode) {
    typedef set<pair<f_ptr_t, string> > type;
    RANDOM_REUSE_UNIQUE(type,
                        f_set,
                        get_random_basis_feature_set(),
                        get_and_feature(mode),
                        [](type vec) {
                            DEBUG_OUT(0, "Used:");
                            for(auto feature_string : vec) {
                                DEBUG_OUT(0, "    " << *(feature_string.first) << " / " <<
                                          feature_string.second);
                            }
                        }
        );
    f_ptr_t and_feature(new AndFeature(f_set.begin()->first));
    for(auto idx_f : util::enumerate(f_set)) {
        if(idx_f.first==0) {
            continue;
        }
        and_feature = f_ptr_t(new AndFeature(and_feature, idx_f.second.first));
    }
    return and_feature;
}
