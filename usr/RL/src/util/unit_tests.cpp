#include "util.h"

#include <gtest/gtest.h>

#include <array>
#include <vector>
#include <list>
#include <deque>

#include <string>

#include <lemon/list_graph.h>

#include <util/softmax.h>
#include <util/util.h>
#include <util/return_tuple.h>
#include <util/pretty_printer.h>
#include <util/template_lib.h>
#include <util/graph_plotting.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::array;
using std::tuple;
using std::cout;
using std::endl;
using std::string;

using util::log_add_exp;

static int counter;

// test log add exp
void test_log_add_exp(double t1, double t2) {
    EXPECT_NEAR(exp(log_add_exp(t1,t2)),exp(t1)+exp(t2),1e-6);
}

// function for testing index conversion by reference
template <class C1, class C2>
void test_ND_idx_ref(int max_dim = 10, int max_bound = 10) {
    int dimensions = rand()%max_dim+1;
    C1 ND_index(dimensions);
    C2 ND_bounds(dimensions);
    {
        auto idx_it = ND_index.begin();
        auto bnd_it = ND_bounds.begin();
        while(idx_it!=ND_index.end()) {
            int bound = rand()%max_bound+1;
            *idx_it = rand()%bound;
            *bnd_it = bound;
            ++idx_it;
            ++bnd_it;
        }
    }
    C1 ND_index_return;
    long long int linear_index;
    util::convert_ND_to_1D_index(ND_index, ND_bounds, linear_index);
    util::convert_1D_to_ND_index(linear_index, ND_index_return, ND_bounds);
    ++counter;
    EXPECT_EQ(ND_index, ND_index_return);
    DEBUG_OUT(2, "linear: " << linear_index <<
              " / ND: " << ND_index_return <<
              " / bounds: " << ND_bounds);
}

// function for testing index conversion by value
template <class C1, class C2>
void test_ND_idx_non_ref(int max_dim = 10, int max_bound = 10) {
    int dimensions = rand()%max_dim+1;
    C1 ND_index(dimensions);
    C2 ND_bounds(dimensions);
    {
        auto idx_it = ND_index.begin();
        auto bnd_it = ND_bounds.begin();
        while(idx_it!=ND_index.end()) {
            int bound = rand()%max_bound+1;
            *idx_it = rand()%bound;
            *bnd_it = bound;
            ++idx_it;
            ++bnd_it;
        }
    }
    auto linear_index = util::convert_ND_to_1D_index(ND_index, ND_bounds);
    auto ND_index_return = util::convert_1D_to_ND_index(linear_index, ND_bounds);
    {
        auto idx_it = ND_index.begin();
        auto idx_ret_it = ND_index_return.begin();
        while(idx_it!=ND_index.end()) {
            ++counter;
            EXPECT_EQ(*idx_it, *idx_ret_it);
            ++idx_it;
            ++idx_ret_it;
        }
    }
    DEBUG_OUT(2, "linear: " << linear_index <<
              " / ND: " << ND_index_return <<
              " / bounds: " << ND_bounds);
}

TEST(Util, LogAddExp) {
    test_log_add_exp(2,5);
    test_log_add_exp(-100,100);
    test_log_add_exp(0,0);
}

TEST(Util, SoftMaxLowTemperatureLimit) {
    DEBUG_OUT(1,"Testing extreme values / over flow");
    vector<double> values({-1e100,-1e10,-10,0,10,1e10,1e100});
    vector<double> expect_probs({0,0,0,0,0,0,1});
    {
        // low but finite temperature
        vector<double> probs = util::soft_max(values,double(1e-300));
        DEBUG_OUT(1,"    T=1e-100");
        for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
            EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
            DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
        }
    }
    {
        // zero-temperature
        vector<double> probs = util::soft_max(values,0);
        DEBUG_OUT(1,"    T=0");
        for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
            EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
            DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
        }
    }
}

TEST(Util, SoftMaxHighTemperatureLimit) {
    DEBUG_OUT(1,"High temperature limit");
    vector<double> values({-1000,-100,0,100,1000});
    vector<double> expect_probs({.2,.2,.2,.2,.2});
    {
        // high but finite temperature
        vector<double> probs = util::soft_max(values,1e10);
        DEBUG_OUT(1,"    T=1e10");
        for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
            EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
            DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
        }
    }
    {
        // infinite temperature
        vector<double> probs = util::soft_max(values,std::numeric_limits<double>::infinity());
        DEBUG_OUT(1,"    T=" << std::numeric_limits<double>::infinity());
        for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
            EXPECT_NEAR(expect_probs[idx],probs[idx],1e-6);
            DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
        }
    }
}

TEST(Util, SoftMaxExactValues) {
    DEBUG_OUT(1,"Exact values");
    vector<double> values({-1,0,1});
    vector<double> expect_probs({0.18632372322584759, 0.30719588571849843, 0.50648039105565412});
    vector<double> probs = util::soft_max(values,2);
    DEBUG_OUT(1,"    T=2");
    for(int idx=0; idx<(int)expect_probs.size(); ++idx) {
        EXPECT_NEAR(expect_probs[idx],probs[idx],1e-15);
        DEBUG_OUT(1,"(" << idx << ") : " << values[idx] << " --> " << probs[idx]);
    }
}

TEST(Util, SoftMaxEqualValues) {
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

TEST(Util, Enumerate) {
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

TEST(Util, Range) {
    int first, last;
    bool is_first;

    // implicit start and increment
    is_first = true;
    for(int i : util::Range(10)) {
        if(is_first) {
            first = i;
            is_first = false;
        }
        last = i;
        cout << i << " ";
    }
    cout << endl;
    EXPECT_EQ(first,0);
    EXPECT_EQ(last,9);

    // implicit increment
    is_first = true;
    for(int i : util::Range(-1,10)) {
        if(is_first) {
            first = i;
            is_first = false;
        }
        last = i;
        cout << i << " ";
    }
    cout << endl;
    EXPECT_EQ(first,-1);
    EXPECT_EQ(last,10);

    // everything explicit
    is_first = true;
    for(int i : util::Range(-3,10,3)) {
        if(is_first) {
            first = i;
            is_first = false;
        }
        last = i;
        cout << i << " ";
    }
    cout << endl;
    EXPECT_EQ(first,-3);
    EXPECT_EQ(last,9);

    // only on element
    is_first = true;
    for(int i : util::Range(1,1)) {
        if(is_first) {
            first = i;
            is_first = false;
        }
        last = i;
        cout << i << " ";
    }
    cout << endl;
    EXPECT_EQ(first,1);
    EXPECT_EQ(last,1);
}

TEST(Util, PrettyPrinter) {
    {
        vector<int> v({1,2,3,4,5});
        cout << v << endl;
    }
    {
        vector<vector<int>> v({{1,2,3},{11,22,33},{111,222,333}});
        cout << v << endl;
    }
}

TEST(Util, IndexConversion) {

    // reset counter
    counter = 0;

    // test 100 random conversions for all combinations of std::vector,
    // std::deque, and std::list both in return-by-reference and return-by-value
    // version.
    repeat(100) {
        test_ND_idx_ref<std::vector<int>,
                        std::vector<int>>();
        test_ND_idx_ref<std::deque<int>,
                        std::deque<int>>();
        test_ND_idx_ref<std::list<int>,
                        std::list<int>>();

        test_ND_idx_ref<std::vector<int>,
                        std::list<int>>();
        test_ND_idx_ref<std::deque<int>,
                        std::vector<int>>();
        test_ND_idx_ref<std::list<int>,
                        std::deque<int>>();

        test_ND_idx_ref<std::vector<int>,
                        std::deque<int>>();
        test_ND_idx_ref<std::deque<int>,
                        std::list<int>>();
        test_ND_idx_ref<std::list<int>,
                        std::vector<int>>();

        //----------------------------------------//

        test_ND_idx_non_ref<std::vector<int>,
                            std::vector<int>>();
        test_ND_idx_non_ref<std::deque<int>,
                            std::deque<int>>();
        test_ND_idx_non_ref<std::list<int>,
                            std::list<int>>();

        test_ND_idx_non_ref<std::vector<int>,
                            std::list<int>>();
        test_ND_idx_non_ref<std::deque<int>,
                            std::vector<int>>();
        test_ND_idx_non_ref<std::list<int>,
                            std::deque<int>>();

        test_ND_idx_non_ref<std::vector<int>,
                            std::deque<int>>();
        test_ND_idx_non_ref<std::deque<int>,
                            std::list<int>>();
        test_ND_idx_non_ref<std::list<int>,
                            std::vector<int>>();
    }

    //---------------------------------//
    // versions with initializer lists //
    //---------------------------------//
    int linear_idx;
    // indices
    std::vector<int> v_idx({3,7,3,8,4});
    std::deque<int> d_idx({3,7,3,8,4});
    std::list<int> l_idx({3,7,3,8,4});
    std::tuple<int,int,int,int,int> t_idx{3,7,3,8,4};
    // bounds
    std::vector<int> v_bnd({11,12,13,14,15});
    std::deque<int> d_bnd({11,12,13,14,15});
    std::list<int> l_bnd({11,12,13,14,15});
    // return indices
    std::vector<int> v_return;
    std::deque<int> d_return;
    std::list<int> l_return;
    std::tuple<int,int,int,int,int> t_return;
    // return by reference
    {
        // vector/vector
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // vector/deque
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, d_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, d_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, d_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // vector/list
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, l_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, l_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, v_return, l_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, v_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // deque/vector
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, v_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, v_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, v_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);
        }
        // deque/deque
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);
        }
        // deque/list
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, l_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, l_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, d_return, l_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, d_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(d_idx, d_return);
        }
        // list/vector
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, v_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, v_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, v_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);
        }
        // list/deque
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, d_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, d_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, d_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);
        }
        // list/list
        {
            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // non-init
            util::convert_1D_to_ND_index(linear_idx, l_return, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);


            // init/non-init
            util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15}, linear_idx);
            // init
            util::convert_1D_to_ND_index(linear_idx, l_return, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(l_idx, l_return);
        }
    }

    // return by value
    {
        // vector/vector
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd);
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15});
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // vector/deque
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd);
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15});
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // vector/list
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd);
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15});
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(v_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // deque/vector
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd);
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15});
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // deque/deque
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd);
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15});
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // deque/list
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd);
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15});
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(d_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // list/vector
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd);
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15});
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            v_return = util::convert_1D_to_ND_index(linear_idx, v_bnd);
            ++counter;
            EXPECT_EQ(v_idx, v_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, v_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // list/deque
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd);
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15});
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            d_return = util::convert_1D_to_ND_index(linear_idx, d_bnd);
            ++counter;
            EXPECT_EQ(d_idx, d_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, d_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
        // list/list
        {
            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd);
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15});
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // non-init
            l_return = util::convert_1D_to_ND_index(linear_idx, l_bnd);
            ++counter;
            EXPECT_EQ(l_idx, l_return);


            // init/non-init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, l_bnd);
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // non-init/init
            linear_idx = util::convert_ND_to_1D_index(l_idx, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);

            // init/init
            linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
            // init
            v_return = util::convert_1D_to_ND_index(linear_idx, {11,12,13,14,15});
            ++counter;
            EXPECT_EQ(v_idx, v_return);
        }
    }
    // using wrapper class
    {
        // vector
        linear_idx = util::convert_ND_to_1D_index(v_idx, v_bnd);
        t_return = util::get_ND_index<5>::from(linear_idx, v_bnd);
        EXPECT_EQ(t_return,t_idx);
        // deque
        linear_idx = util::convert_ND_to_1D_index(d_idx, d_bnd);
        t_return = util::get_ND_index<5>::from(linear_idx, d_bnd);
        EXPECT_EQ(t_return,t_idx);
        // list
        linear_idx = util::convert_ND_to_1D_index(l_idx, l_bnd);
        t_return = util::get_ND_index<5>::from(linear_idx, l_bnd);
        EXPECT_EQ(t_return,t_idx);
        // initializer list
        linear_idx = util::convert_ND_to_1D_index({3,7,3,8,4}, {11,12,13,14,15});
        t_return = util::get_ND_index<5>::from(linear_idx, {11,12,13,14,15});
        EXPECT_EQ(t_return,t_idx);
    }

    DEBUG_OUT(1, "Performed " << counter << " checks");

    // reset counter (for future use -- just in case...)
    counter = 0;
}

TEST(Util, ReturnTuple) {

    repeat(100) {

        int rand_int = rand();
        double rand_double = drand48();

        // create named tuple and assign separately
        {

            NAMED_RETURN_TUPLE(tt, int, i, double, d);
            tt = std::make_tuple(rand_int, rand_double);
            EXPECT_EQ(rand_int, i);
            EXPECT_EQ(rand_double, d);
            EXPECT_EQ(i, std::get<0>(tt));
            EXPECT_EQ(d, std::get<1>(tt));
        }

        // create named tuple and assign in one step
        {
            NAMED_RETURN_TUPLE(tt, int, i, double, d);
            tt = std::make_tuple(rand_int, rand_double);
            EXPECT_EQ(rand_int, i);
            EXPECT_EQ(rand_double, d);
            EXPECT_EQ(i, std::get<0>(tt));
            EXPECT_EQ(d, std::get<1>(tt));
        }

        // create anonymous tuple for assignment only
        {
            RETURN_TUPLE(int, i, double, d) = std::make_tuple(rand_int, rand_double);
            EXPECT_EQ(rand_int, i);
            EXPECT_EQ(rand_double, d);
        }

        // create variables separately and assign as tuple
        {
            int i; double d;
            return_tuple::t(i,d) = std::make_tuple(rand_int, rand_double);
            EXPECT_EQ(rand_int, i);
            EXPECT_EQ(rand_double, d);
        }

    }
}

TEST(Util, ArrayToTuple) {
    using namespace template_lib;
    {
        array<int,1> a = {11};
        tuple<int> t = array_to_tuple(a);
        tuple<int> t_ret(11);
        EXPECT_EQ(t, t_ret);
    }
    {
        array<int,2> a = {11,22};
        tuple<int,int> t = array_to_tuple(a);
        tuple<int,int> t_ret(11,22);
        EXPECT_EQ(t, t_ret);
    }
    {
        array<int,3> a = {11,22,33};
        tuple<int,int,int> t = array_to_tuple(a);
        tuple<int,int,int> t_ret(11,22,33);
        EXPECT_EQ(t, t_ret);
    }
    {
        array<int,4> a = {11,22,33,44};
        tuple<int,int,int,int> t = array_to_tuple(a);
        tuple<int,int,int,int> t_ret(11,22,33,44);
        EXPECT_EQ(t, t_ret);
    }
}

TEST(Util, GraphToPdf) {

    //! [graph_to_pdf example]

    using namespace lemon;

    // construct the graph
    ListDigraph graph;
    auto n1 = graph.addNode();
    auto n2 = graph.addNode();
    auto n3 = graph.addNode();
    graph.addArc(n1,n2);
    graph.addArc(n1,n3);

    // get a node map (only filling the first entry)
    ListDigraph::NodeMap<QString> node_map(graph);
    for(ListDigraph::NodeIt node(graph); node!=INVALID; ++node) {
        node_map[node] = "color=red";
        break;
    }

    // get an arc map (only filling the first entry)
    ListDigraph::ArcMap<QString> arc_map(graph);
    for(ListDigraph::ArcIt arc(graph); arc!=INVALID; ++arc) {
        arc_map[arc] = "style=solid";
        break;
    }

    // plot the graph
    util::plot_graph("graph.pdf", graph, "shape=square", &node_map, "style=dashed", &arc_map);

    //! [graph_to_pdf example]

    EXPECT_EQ(0,system("dot -V")) << "Could not find 'dot' command for generating graphs.";
    EXPECT_EQ(0,remove("graph.pdf")) << "Graph 'graph.pdf' was not generated.";
}
