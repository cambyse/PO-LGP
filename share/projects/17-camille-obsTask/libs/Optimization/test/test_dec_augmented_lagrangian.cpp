#include <decentralized_aula.h>
#include <gtest/gtest.h>
#include "functions.cpp"

constexpr double eps_s = 0.02;

TEST(DecentralizedAugmentedLagrangian,DecAulaBattlingADMMoverY) {
  arr x{0.0, 0.0, 0.0, 0.0};
  arr dual;

  const arr center0{1.0, 1.5, 1.0, 1.0};
  const arr center1{1.0, 0.5, 1.0, 1.0};

  auto pb0 = std::make_shared<Distance4DMasked>(center0, arr{sqrt(1.0/2.0), sqrt(1.0/4.0), 0.0, 1.0});
  auto pb1 = std::make_shared<Distance4DMasked>(center1, arr{sqrt(1.0/2.0), sqrt(3.0/4.0), 1.0, 0.0});

  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  DecOptConstrained opt(x, dual, pbs);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
  EXPECT_NEAR(1.0, x(3), eps_s);
}


TEST(DecentralizedAugmentedLagrangian,DecAula4D3Problems) {
  arr x{0.0, 0.0, 0.0, 0.0};
  arr dual;

  const arr center{1.0, 1.0, 1.0, 1.0};

  auto pb0 = std::make_shared<Distance4DMasked>(center, arr{sqrt(1.0/3.0), 1.0, 0.0, 0.0});
  auto pb1 = std::make_shared<Distance4DMasked>(center, arr{sqrt(1.0/3.0), 0.0, 1.0, 0.0});
  auto pb2 = std::make_shared<Distance4DMasked>(center, arr{sqrt(1.0/3.0), 0.0, 0.0, 1.0});

  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);
  pbs.push_back(pb2);

  DecOptConstrained opt(x, dual, pbs);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
  EXPECT_NEAR(1.0, x(3), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaWithDecomposedProblem) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  auto pb0 = std::make_shared<Distance3DDecompXY>(arr{1.0, 1.0, 1.0});
  auto pb1 = std::make_shared<Distance3DDecompXZ>(arr{1.0, 1.0, 1.0});
  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  DecOptConstrained opt(x, dual, pbs);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaWithOneProblem) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  auto pb = std::make_shared<Distance3DDecompXY>(arr{1.0, 1.0, 1.0});
  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb);

  DecOptConstrained opt(x, dual, pbs);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(0.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, Distance3DTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3D pb(arr{1.0, 1.0, 1.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, Distance3DDecompXYTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3DDecompXY pb(arr{1.0, 1.0, 1.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(0.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, Distance3DDecompXZTestHG) {
  arr x{0.0, 0.0, 0.0};
  arr dual;

  Distance3DDecompXZ pb(arr{1.0, 1.0, 1.0});

  OptConstrained opt(x, dual, pb);
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(0.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}


//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
