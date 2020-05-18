#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <gtest/gtest.h>

#include "functions.cpp"

constexpr double eps = 0.0001;
constexpr double eps_s = 0.01;

TEST(AugmentedLagrangian, SimpleParabolTestG) {
  arr x{1.0};
  arr dual; //dual

  Parabol pb;

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(-0.5, x(0), eps_s);
}

TEST(AugmentedLagrangian, Distance2DTestHG) {
  arr x{1.0, 2.0};
  arr dual; //dual

  Distance2D pb;

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
}

TEST(AugmentedLagrangian, SimpleParabolWithFTerm) {
  arr x{1.0};
  arr dual; //dual

  ParabolWithFTerm pb;

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.5, x(0), eps_s);
}

TEST(AugmentedLagrangian, MultipleRuns) {
  arr x{1.0, 2.0};
  arr dual;

  Distance2D pb;

  OptConstrained opt(x, dual, pb);
  std::cout << "phase 1" << std::endl;
  opt.run();
  std::cout << "phase 2" << std::endl;
  x -= 0.3;
  OptConstrained opt2(x, dual, pb);
  opt2.run();
  std::cout << "phase 3" << std::endl;
  OptConstrained opt3(x, dual, pb);
  opt3.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
