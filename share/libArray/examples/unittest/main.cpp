#include <MT/array.h>
#include "gtest/gtest.h"

GTEST_API_ int main(int argc, char** argv) {
  linkArray();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
