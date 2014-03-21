#include <gtest/gtest.h>

#include "../util/util.h"

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

#include "RandomElements.h"

#include "../util/ProgressBar.h"

#include <vector>
#include <list>
#include <memory> // shared_ptr
#include <sstream>

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using std::vector;
using std::list;
using std::string;
using std::stringstream;
using std::shared_ptr;
using std::dynamic_pointer_cast;
using util::random_select;

typedef AbstractAction::ptr_t action_ptr_t;
typedef AbstractObservation::ptr_t observation_ptr_t;
typedef AbstractReward::ptr_t reward_ptr_t;
typedef Feature::const_feature_ptr_t f_ptr_t;

int get_time_delay();
f_ptr_t get_basis_feature();
f_ptr_t get_const_feature();
f_ptr_t get_action_feature();
f_ptr_t get_observation_feature();
f_ptr_t get_reward_feature();
f_ptr_t get_and_feature();

TEST(FeatureTest, SharedPtr) {

    // vector holding pointers to basis features
    vector<f_ptr_t> basis_features;

    // Construct basis features
    basis_features.push_back(ConstFeature::create(3.5));
    basis_features.push_back(ActionFeature::create(action_ptr_t(new AbstractAction()), 0));
    basis_features.push_back(ObservationFeature::create(observation_ptr_t(new AbstractObservation()), 0));
    basis_features.push_back(RewardFeature::create(reward_ptr_t(new AbstractReward()), 0));

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
}

TEST(FeatureTest, ComparisonAndOrdering) {

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
                    EXPECT_TRUE(*f1<*f2 || *f2<*f1) << *f1 << "!=" << *f2 << " violated by ordering operator";
                }
            }
            ProgressBar::print(counter++, number_of_features);
        }
        ProgressBar::terminate();
    }

    // check ordering via stupid sorting
    {
        int counter = 0;
        list<f_ptr_t> sorted_feature_list;
        ProgressBar::init("Checking Ordering: ");
        for(f_ptr_t f_unsorted : feature_vector) {
            auto insert_before = sorted_feature_list.begin();   // first element that is not smaller
            auto some_elem_after = sorted_feature_list.begin(); // all other elements after that
            bool found = false;
            while(insert_before!=sorted_feature_list.end() && some_elem_after!=sorted_feature_list.end()) {
                if(!found && **insert_before<*f_unsorted) {
                    ++insert_before;
                } else {
                    if(!found) {
                        found = true;
                        some_elem_after = insert_before;
                    }
                    ++some_elem_after;
                    if(some_elem_after!=sorted_feature_list.end()) {
                        EXPECT_FALSE(**some_elem_after<*f_unsorted) << **some_elem_after << " should be larger or equal to " << *f_unsorted;
                    }
                }
            }
            sorted_feature_list.insert(insert_before,f_unsorted);
            ProgressBar::print(counter++, number_of_features);
        }
        ProgressBar::terminate();

        // check sorting
        for(auto low_elem=sorted_feature_list.begin(); low_elem!=sorted_feature_list.end(); ++low_elem) {
            for(auto high_elem=low_elem; high_elem!=sorted_feature_list.end(); ++high_elem) {
                EXPECT_FALSE(**high_elem<**low_elem) << **high_elem << "<" << **low_elem;
            }
        }

        // print
        if(DEBUG_LEVEL>1) {
            for(f_ptr_t sorted_feature : sorted_feature_list) {
                DEBUG_OUT(0,"    " << *sorted_feature);
            }
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
