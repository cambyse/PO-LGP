#include "util.h"

#include <gtest/gtest.h>

#include <array>
#include <vector>
#include <list>
#include <deque>

#include <util/util.h>
#include <util/return_tuple_macros.h>
#include <util/pretty_printer.h>
#include <util/template_lib.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::array;
using std::tuple;

static int counter;

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
    // bounds
    std::vector<int> v_bnd({11,12,13,14,15});
    std::deque<int> d_bnd({11,12,13,14,15});
    std::list<int> l_bnd({11,12,13,14,15});
    // return indices
    std::vector<int> v_return;
    std::deque<int> d_return;
    std::list<int> l_return;
    std::array<int,5> a_return;
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

    DEBUG_OUT(1, "Performed " << counter << " checks");

    // reset counter (for future use -- just in case...)
    counter = 0;
}

TEST(Util, ReturnTuple) {
    int r;

    // create named tuple separately
    {
        r = rand()%10;
        TT1(tt, int, i);
        tt = std::make_tuple(r);
        EXPECT_EQ(r, i);
        EXPECT_EQ(i, std::get<0>(tt));
    }

    // create named tuple and assign
    {
        r = rand()%10;
        TT1(tt, int, i) = std::make_tuple(r);
        EXPECT_EQ(r, i);
        EXPECT_EQ(i, std::get<0>(tt));
    }

    // create anonymous tuple for assignment only
    {
        r = rand()%10;
        T1(int, i) = std::make_tuple(r);
        DEBUG_OUT(r,i);
    }

    // and for 10-tuple
    {
        r = rand()%10;
        TT10(tt, int, i1, int, i2, int, i3, int, i4, int, i5,
             int, i6, int, i7, int, i8, int, i9, int, i10);
        tt = std::make_tuple(r,r,r,r,r,r,r,r,r,r);
        EXPECT_EQ(r, i1);
        EXPECT_EQ(r, i2);
        EXPECT_EQ(r, i3);
        EXPECT_EQ(r, i4);
        EXPECT_EQ(r, i5);
        EXPECT_EQ(r, i6);
        EXPECT_EQ(r, i7);
        EXPECT_EQ(r, i8);
        EXPECT_EQ(r, i9);
        EXPECT_EQ(r, i10);
        EXPECT_EQ(i1, std::get<0>(tt));
        EXPECT_EQ(i2, std::get<1>(tt));
        EXPECT_EQ(i3, std::get<2>(tt));
        EXPECT_EQ(i4, std::get<3>(tt));
        EXPECT_EQ(i5, std::get<4>(tt));
        EXPECT_EQ(i6, std::get<5>(tt));
        EXPECT_EQ(i7, std::get<6>(tt));
        EXPECT_EQ(i8, std::get<7>(tt));
        EXPECT_EQ(i9, std::get<8>(tt));
        EXPECT_EQ(i10, std::get<9>(tt));
    }

    using namespace template_lib;
    {
        array<int,1> a = {11};
        tuple<int> t = array_to_tuple(a);
        DEBUG_OUT(0, t);
    }
    {
        array<int,2> a = {11,22};
        tuple<int,int> t = array_to_tuple(a);
        DEBUG_OUT(0, t);
    }
    {
        array<int,3> a = {11,22,33};
        tuple<int,int,int> t = array_to_tuple(a);
        DEBUG_OUT(0, t);
    }
    {
        std::tuple<int,int,int> tt{2,3,4};
    }
}
