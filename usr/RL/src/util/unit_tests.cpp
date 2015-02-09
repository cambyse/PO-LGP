#include "util.h"

#include <gtest/gtest.h>

#include "../util/util.h"

#include "../util/debug.h"

using std::vector;

TEST(Util, SomeTest) {
    vector<int> index({1,4,0,21,8});
    vector<int> bounds({100,100,100,100,100});
    long long int idx;
    util::ND_index_to_linear(index, bounds, idx);
    auto index2 = util::linear_to_ND_index(idx, bounds);
    DEBUG_OUT(0, index << " --> " << idx << " --> " << index2);
}
