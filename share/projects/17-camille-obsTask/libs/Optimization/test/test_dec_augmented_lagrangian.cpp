#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <gtest/gtest.h>
#include "functions.cpp"

constexpr double eps = 0.0001;
constexpr double eps_s = 0.01;

TEST(ScalarFunction, SimpleParabolTestG) {
  arr x{1.0};
  arr dual; //dual

  Parabol pb;

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(-0.5, x(0), eps_s);
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
