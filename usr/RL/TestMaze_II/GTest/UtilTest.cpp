#include <gtest/gtest.h>

#include <string>

#include "../util/util.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using std::vector;
using std::cout;
using std::endl;
using std::string;

using util::log_add_exp;

void test_log_add_exp(double t1, double t2) {
    EXPECT_NEAR(exp(log_add_exp(t1,t2)),exp(t1)+exp(t2),1e-6);
}

TEST(UtilTest, LogAddExp) {
    test_log_add_exp(2,5);
    test_log_add_exp(-100,100);
    test_log_add_exp(0,0);
}

TEST(UtilTest, SoftMax) {
    // extreme value / over flow
    {
        DEBUG_OUT(1,"Testing extreme values / over flow");
        vector<double> values({-1000,-100,-10,0,10,100,1000});
        vector<double> expect_probs({0,0,0,0,0,0,1});
        vector<double> probs = util::soft_max(values,1e-10);
        DEBUG_OUT(1,"    T=1e-10");
        for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
            EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
            DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
        }
    }

    // high temperature limit
    {
        DEBUG_OUT(1,"High temperature limit");
        vector<double> values({-1000,-100,0,100,1000});
        vector<double> expect_probs({.2,.2,.2,.2,.2});
        vector<double> probs = util::soft_max(values,1e10);
        DEBUG_OUT(1,"    T=1e10");
        for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
            EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
            DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
        }
    }

    // equal values
    {
        DEBUG_OUT(1,"Testing equally distributed values");
        vector<double> probs;
        vector<double> expect_probs({.2,.2,.2,.2,.2});

        {
            DEBUG_OUT(1,"    {1,1,1,1,1}");
            vector<double> values({1,1,1,1,1});
            probs = util::soft_max(values,1e-10);
            DEBUG_OUT(1,"    T=1e-10");
            for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
                EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
                DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
            }
            probs = util::soft_max(values,1);
            DEBUG_OUT(1,"    T=1");
            for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
                EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
                DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
            }
        }

        {
            DEBUG_OUT(1,"    {0,0,0,0,0}");
            vector<double> values({0,0,0,0,0});
            probs = util::soft_max(values,1e-10);
            DEBUG_OUT(1,"    T=1e-10");
            for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
                EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
                DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
            }
            probs = util::soft_max(values,1);
            DEBUG_OUT(1,"    T=1");
            for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
                EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
                DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
            }
        }

        {
            DEBUG_OUT(1,"    {-1,-1,-1,-1,-1}");
            vector<double> values({-1,-1,-1,-1,-1});
            probs = util::soft_max(values,1e-10);
            DEBUG_OUT(1,"    T=1e-10");
            for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
                EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
                DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
            }
            probs = util::soft_max(values,1);
            DEBUG_OUT(1,"    T=1");
            for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
                EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
                DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
            }
        }
    }
}

TEST(UtilTest, Enumerate) {
    vector<string> vec = {"1st elem", "2nd elem", "3rd elem", "4th elem"};
    int counter;
    // simple form
    counter = 0;
    for(auto idx_elem : util::enumerate(vec)) {
        EXPECT_EQ(idx_elem.first, counter);
        EXPECT_EQ(idx_elem.second, vec[counter]);
        DEBUG_OUT(1, idx_elem.first << " / " << idx_elem.second);
        ++counter;
    }
    // with specific value for start and increment
    counter = -10;
    for(auto idx_elem : util::enumerate(vec, counter, 2)) {
        EXPECT_EQ(idx_elem.first, counter);
        DEBUG_OUT(1, idx_elem.first << " / " << idx_elem.second);
        counter += 2;
    }
    // assigning
    for(auto idx_elem : util::enumerate(vec)) {
        string new_elem;
        repeat(idx_elem.first) {
            new_elem += "X";
        }
        idx_elem.second = new_elem;
    }
    for(auto idx_elem : util::enumerate(vec)) {
        string new_elem;
        repeat(idx_elem.first) {
            new_elem += "X";
        }
        EXPECT_EQ(idx_elem.second, new_elem);
        DEBUG_OUT(1, idx_elem.first << " / " << idx_elem.second);
    }
}
