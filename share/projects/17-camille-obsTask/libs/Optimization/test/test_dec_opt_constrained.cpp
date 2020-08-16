#include <Optimization/decentralized_optimizer.h>
#include <gtest/gtest.h>
#include "functions.cpp"

constexpr double eps_s = 0.02;

using T = ConstrainedProblem;
/*
TEST(DecentralizedAugmentedLagrangian, DecAulaBattlingADMMoverYSequential) {
  arr x{0.0, 0.0, 0.0};

  const arr center0{1.0, 1.5, 1.0};
  const arr center1{1.0, 0.5, 1.0};

  auto pb0 = std::make_shared<Distance3D>(center0, arr{1.0, 1.5, 1.0});
  auto pb1 = std::make_shared<Distance3D>(center1, arr{1.0, 0.5, 1.0});

  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  DecOptConstrained<T> opt(x, pbs, {}, DecOptConfig(SEQUENTIAL, false));

  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_TRUE(x(1) > 1.0);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaWithCompressedProblemSequential) {
  // distance 3d decomposed in 2x dist 2d
  arr x{0.0, 1.0, 2.0};

  const arr center0 {1.0, 1.0};
  const arr center1 {1.0, 0.5};

  auto pb0 = std::make_shared<Distance2D>(center0, arr{sqrt(0.5), 1.0});
  auto pb1 = std::make_shared<Distance2D>(center1, arr{sqrt(0.5), 1.0});
  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  std::vector<arr> masks;
  masks.push_back(arr{1.0, 1.0, 0.0});
  masks.push_back(arr{1.0, 0.0, 1.0});

  DecOptConstrained<T> opt(x, pbs, masks, DecOptConfig(SEQUENTIAL, true));

  EXPECT_EQ((intA{0, 1}), opt.vars[0]);
  EXPECT_EQ((intA{0, 2}), opt.vars[1]);

  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(0.5, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaBattlingADMMoverYSequentialAndParallel) {
  arr x{0.0, 0.0, 0.0};

  const arr center0{1.0, 1.5, 1.0};
  const arr center1{1.0, 0.5, 1.0};

  auto pb0 = std::make_shared<Distance3D>(center0, arr{1.0, 1.5, 1.0});
  auto pb1 = std::make_shared<Distance3D>(center1, arr{1.0, 0.5, 1.0});

  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  DecOptConstrained<T> opt(x, pbs, {}, DecOptConfig(FIRST_ITERATION_SEQUENTIAL_THEN_PARALLEL, false));

  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_TRUE(x(1) > 1.0);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaWithCompressedProblemUsingVars) {
  // distance 3d decomposed in 2x dist 2d
  arr x{0.0, 1.0, 2.0};

  const arr center {1.0, 1.0};

  auto pb0 = std::make_shared<Distance2D>(center, arr{sqrt(0.5), 1.0});
  auto pb1 = std::make_shared<Distance2D>(center, arr{sqrt(0.5), 0.0});
  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  std::vector<arr> masks;
  masks.push_back(arr{1.0, 1.0, 0.0});
  masks.push_back(arr{1.0, 0.0, 1.0});

  DecOptConstrained<T> opt(x, pbs, masks, DecOptConfig(PARALLEL, true));

  EXPECT_EQ((intA{0, 1}), opt.vars[0]);
  EXPECT_EQ((intA{0, 2}), opt.vars[1]);

  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaWithDecomposedProblemUsingMasks) {
  arr x{0.0, 0.0, 0.0};

  const arr center {1.0, 1.0, 1.0};

  auto pb0 = std::make_shared<Distance3D>(center, arr{sqrt(0.5), 1.0, 0.0});
  auto pb1 = std::make_shared<Distance3D>(center, arr{sqrt(0.5), 0.0, 1.0});
  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  std::vector<arr> masks;
  masks.push_back(arr{1.0, 1.0, 0.0});
  masks.push_back(arr{1.0, 0.0, 1.0});

  DecOptConstrained<T> opt(x, pbs, masks, DecOptConfig(PARALLEL, false));
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaWithDecomposedProblem) {
  arr x{0.0, 0.0, 0.0};

  const arr center {1.0, 1.0, 1.0};

  auto pb0 = std::make_shared<Distance3D>(center, arr{sqrt(0.5), 1.0, 0.0});
  auto pb1 = std::make_shared<Distance3D>(center, arr{sqrt(0.5), 0.0, 1.0});
  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  DecOptConstrained<T> opt(x, pbs, {}, DecOptConfig(PARALLEL, false));
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaBattlingADMMoverY) {
  arr x{0.0, 0.0, 0.0};

  const arr center0{1.0, 1.5, 1.0};
  const arr center1{1.0, 0.5, 1.0};

  auto pb0 = std::make_shared<Distance3D>(center0, arr{1.0, 1.5, 1.0});
  auto pb1 = std::make_shared<Distance3D>(center1, arr{1.0, 0.5, 1.0});

  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);

  DecOptConstrained<T> opt(x, pbs, {}, DecOptConfig(PARALLEL, false));

  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_TRUE(x(1) > 1.0);
  EXPECT_NEAR(1.0, x(2), eps_s);
}


TEST(DecentralizedAugmentedLagrangian, DecAula4D3Problems) {
  arr x{0.0, 0.0, 0.0, 0.0};

  const arr center{1.0, 1.0, 1.0, 1.0};

  auto pb0 = std::make_shared<Distance4D>(center, arr{sqrt(1.0/3.0), 1.0, 0.0, 0.0});
  auto pb1 = std::make_shared<Distance4D>(center, arr{sqrt(1.0/3.0), 0.0, 1.0, 0.0});
  auto pb2 = std::make_shared<Distance4D>(center, arr{sqrt(1.0/3.0), 0.0, 0.0, 1.0});

  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb0);
  pbs.push_back(pb1);
  pbs.push_back(pb2);

  DecOptConstrained<T> opt(x, pbs, {}, DecOptConfig(PARALLEL, false));
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(1.0, x(2), eps_s);
  EXPECT_NEAR(1.0, x(3), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, DecAulaWithOneProblem) {
  arr x{0.0, 0.0, 0.0};

  auto pb = std::make_shared<Distance3D>(arr{1.0, 1.0, 1.0}, arr{1.0, 1.0, 0.0});
  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb);

  DecOptConstrained<T> opt(x, pbs, {}, DecOptConfig(PARALLEL, false));
  opt.run();

  EXPECT_NEAR(0.0, x(0), eps_s);
  EXPECT_NEAR(1.0, x(1), eps_s);
  EXPECT_NEAR(0.0, x(2), eps_s);
}

TEST(DecentralizedAugmentedLagrangian, CallbackCall) {
  arr x{0.0, 0.0, 0.0};

  auto pb = std::make_shared<Distance3D>(arr{1.0, 1.0, 1.0}, arr{1.0, 1.0, 0.0});
  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;
  pbs.push_back(pb);

  bool called=false;
  DecOptConfig options(PARALLEL, false);
  options.callback = [&called]()
  {
    called = true;
  };

  DecOptConstrained<T> opt(x, pbs, {}, options);
  opt.run();

  EXPECT_TRUE(called);
}*/

// WARM START
TEST(DecentralizedAugmentedLagrangian, CallbackCallWarmStart) {
  arr x{5.0, 0.0};

  std::vector<std::shared_ptr<ConstrainedProblem>> pbs;

  auto pb1 = std::make_shared<Valley2DSideWaysDecomposed0>();
  auto pb2 = std::make_shared<Valley2DSideWaysDecomposed1>();
  pbs.push_back(pb1);
  pbs.push_back(pb2);

  uint n_called=0;
  uint n1, n2;
  n1 = n2;
  DecOptConfig options(PARALLEL, false);
  options.opt.aulaMuInc = 1.0;
  options.callback = [&n_called]()
  {
    n_called++;
  };

  DualState state;
  {
  DecOptConstrained<T> opt(x, pbs, {}, options);
  opt.run();
  state = opt.get_dual_state();
  n1 = n_called;
  n_called = 0;
  }

  std::cout << "----------------------------" << std::endl;

  x(0)+=1;
  pb1->xstart=1.0;
  {
  DecOptConstrained<T> opt(x, pbs, {}, options);
  opt.set_dual_state(state);
  opt.run();
  }
  n2 = n_called;

  //EXPECT_TRUE(newton2.front() - newton1.front() < newton1.front());
  EXPECT_TRUE(n1 > n2);
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
