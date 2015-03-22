#include <gtest/gtest.h>

int main(int argc, char **argv) {
    srand(time(nullptr));
    srand48(time(nullptr));
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
