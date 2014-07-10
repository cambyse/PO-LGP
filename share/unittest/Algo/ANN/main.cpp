#include <Algo/ann.h>
#include <Core/array.h>
#include <gtest/gtest.h>

void test_ann(uint num) {
  ANN ann;
  for (uint i = 0; i < num; ++i) {
    arr next = rand(3,1);
    next.reshape(3);
    ann.append(next);

    arr search = rand(3,1);
    search.reshape(3);
    ann.getNN(search, .01, true);
  }
}

GTEST_TEST(AlgosTest, testANNBig) {
  test_ann(10000);
}

GTEST_TEST(AlgosTest, testANNSmall) {
  test_ann(100);
}


GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
