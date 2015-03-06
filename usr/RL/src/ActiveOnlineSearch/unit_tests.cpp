#include <gtest/gtest.h>

#include "../util/util.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

#include <ActiveOnlineSearch/ReverseAccumulation.h>

TEST(ActiveOnlineSearch, ReverseAccumulation) {
    ReverseAccumulation ra;
    ra.assign_values({0.1,3,30});
    ra.propagate_values();
    ra.plot_graph("graph.pdf");
}
