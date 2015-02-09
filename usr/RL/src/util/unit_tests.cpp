#include "util.h"

#include <gtest/gtest.h>

#include <vector>
#include <list>
#include <deque>

#include "../util/util.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using std::vector;

template <class C1, class C2>
void test_ND_idx_sequence_ref(int max_dim = 10, int max_bound = 10) {
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
    util::ND_index_to_linear(ND_index, ND_bounds, linear_index);
    util::linear_to_ND_index(linear_index, ND_index_return, ND_bounds);
    EXPECT_EQ(ND_index, ND_index_return);
    DEBUG_OUT(1, "linear: " << linear_index <<
              " / ND: " << ND_index_return <<
              " / bounds: " << ND_bounds);
}

template <class C1, class C2>
void test_ND_idx_sequence_non_ref(int max_dim = 10, int max_bound = 10) {
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
    auto linear_index = util::ND_index_to_linear(ND_index, ND_bounds);
    auto ND_index_return = util::linear_to_ND_index(linear_index, ND_bounds);
    {
        auto idx_it = ND_index.begin();
        auto idx_ret_it = ND_index_return.begin();
        while(idx_it!=ND_index.end()) {
            EXPECT_EQ(*idx_it, *idx_ret_it);
            ++idx_it;
            ++idx_ret_it;
        }
    }
    DEBUG_OUT(1, "linear: " << linear_index <<
              " / ND: " << ND_index_return <<
              " / bounds: " << ND_bounds);
}


TEST(Util, MultidimensionalIndices) {
    repeat(100) {
        test_ND_idx_sequence_ref<std::vector<int>,
                                 std::vector<int>>();
        test_ND_idx_sequence_ref<std::deque<int>,
                                 std::deque<int>>();
        test_ND_idx_sequence_ref<std::list<int>,
                                 std::list<int>>();

        test_ND_idx_sequence_ref<std::vector<int>,
                                 std::list<int>>();
        test_ND_idx_sequence_ref<std::deque<int>,
                                 std::vector<int>>();
        test_ND_idx_sequence_ref<std::list<int>,
                                 std::deque<int>>();

        test_ND_idx_sequence_ref<std::vector<int>,
                                 std::deque<int>>();
        test_ND_idx_sequence_ref<std::deque<int>,
                                 std::list<int>>();
        test_ND_idx_sequence_ref<std::list<int>,
                                 std::vector<int>>();

        //----------------------------------------//

        test_ND_idx_sequence_non_ref<std::vector<int>,
                                     std::vector<int>>();
        test_ND_idx_sequence_non_ref<std::deque<int>,
                                     std::deque<int>>();
        test_ND_idx_sequence_non_ref<std::list<int>,
                                     std::list<int>>();

        test_ND_idx_sequence_non_ref<std::vector<int>,
                                     std::list<int>>();
        test_ND_idx_sequence_non_ref<std::deque<int>,
                                     std::vector<int>>();
        test_ND_idx_sequence_non_ref<std::list<int>,
                                     std::deque<int>>();

        test_ND_idx_sequence_non_ref<std::vector<int>,
                                     std::deque<int>>();
        test_ND_idx_sequence_non_ref<std::deque<int>,
                                     std::list<int>>();
        test_ND_idx_sequence_non_ref<std::list<int>,
                                     std::vector<int>>();
    }
}
