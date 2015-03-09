#include <gtest/gtest.h>

#include "../util/util.h"
#include "../util/ND_vector.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

#include <ActiveOnlineSearch/ReverseAccumulation.h>

using namespace ND_vector;
using util::Range;

TEST(ActiveOnlineSearch, ReverseAccumulation) {
    ReverseAccumulation ra;
    ra.compute_values({0.1,3,30});
    auto output_vals = ra.get_output_values();

    ra.forward_accumulation({0.1,3,30},{1,0,0});
    auto output_diff_1 = ra.get_output_differentials();
    ra.forward_accumulation({0.1,3,30},{0,1,0});
    auto output_diff_2 = ra.get_output_differentials();
    ra.forward_accumulation({0.1,3,30},{0,0,1});
    auto output_diff_3 = ra.get_output_differentials();

    ra.reverse_accumulation({0.1,3,30},{1,0});
    auto input_diff_1 = ra.get_input_differentials();
    ra.reverse_accumulation({0.1,3,30},{0,1});
    auto input_diff_2 = ra.get_input_differentials();

    vec_double_2D input_output_diff({output_diff_1, output_diff_2, output_diff_3});
    vec_double_2D output_input_diff({input_diff_1, input_diff_2});

    for(int input_idx : Range(3)) {
        for(int output_idx : Range(2)) {
            EXPECT_NEAR(input_output_diff[input_idx][output_idx],
                        output_input_diff[output_idx][input_idx], 1e-10);
        }
    }

}
