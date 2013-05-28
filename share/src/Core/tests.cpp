#include "array.h"
#include "gtest/gtest.h"

TEST(ArrayTest, testOnes) {
  arr a = ones(5, 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_EQ(a(i, j), 1);
    }
  }
}

TEST(ArrayTest, testZeros) {
  arr a = zeros(5, 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_EQ(a(i, j), 0);
    }
  }
}

TEST(ArrayTest, testEye) {
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

