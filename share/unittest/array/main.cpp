#include<MT/array.h>
#include "gtest/gtest.h"

// GoogleTest Resources:
// - Primer: https://code.google.com/p/googletest/wiki/V1_6_Primer
// - Examples: https://code.google.com/p/googletest/wiki/V1_6_Samples
// - Advanced: https://code.google.com/p/googletest/wiki/V1_6_AdvancedGuide

// - ArrayTest is the name of the test class.
// - a test class can have multiple tests. in this case the test is called
//   testMatlabFunctionality
TEST(ArrayMatlabTest, testOnes) {
  arr a = ones(5, 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_EQ(a(i, j), 1);
    }
  }
}

TEST(ArrayMatlabTest, testZeros) {
  arr a = zeros(5, 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_EQ(a(i, j), 0);
    }
  }
}

TEST(ArrayMatlabTest, testEye) {
  arr a = eye(5, 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      if (i == j) {
        EXPECT_EQ(a(i, j), 1) << "diagonal should be 1";
      } else {
        EXPECT_EQ(a(i, j), 0) << "non-diagonal should be 0";
      }
    }
  }
}


// TODO The main should be moved to an extenal file which runs all unittests
// and creates a test report which can be used by jenkins.
GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
