#include <gtest/gtest.h>

#include "SandBox.h"

#include <vector>
#include <string>

#include "../util/debug.h"

using std::vector;
using std::string;

vector<string> vec = {"aaa", "bbb", "ccc", "ddd"};

TEST(SandBox, Enumerate) {
    for(auto i : enumerate(vec)) {
        DEBUG_OUT(0, i.first << " / " << i.second);
    }
}
