#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <Optimization/decentralized_optimizer.h>
#include <Optimization/qp_lagrangian.h>
#include <gtest/gtest.h>

#include "functions.cpp"

constexpr double eps = 0.0001;
constexpr double eps_s = 0.01;

namespace
{
  DecOptConfig buildOptions()
  {
    DecOptConfig options(PARALLEL, false, NOOPT, true);
    options.opt.stopTolerance = 0.001;
    return options;
  }
}

TEST(QP, Unconstrained) {
  arr x = arr(1); x(0) = 0;

  arr P(1,1); P(0,0) = 2;
  arr q(1); q(0) = -1;

  auto qp = std::make_shared<QP_Problem>(P, q, NoArr, NoArr);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp);

  auto options = buildOptions();
  DecOptConstrained<QP_Problem> opt(x, pbs, {}, options);

  opt.run();

  EXPECT_NEAR(0.5, x(0), eps_s);
}

TEST(QP, OneDimOneConstrained) {
  arr x = arr(1); x(0) = 0;

  arr P(1,1); P(0,0) = 2;
  arr q(1); q(0) = -1;
  arr K(1, 1); K(0, 0) = 1;
  arr u(1); u(0) = 0.2;

  auto qp = std::make_shared<QP_Problem>(P, q, K, u);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp);

  auto options = buildOptions();
  DecOptConstrained<QP_Problem> opt(x, pbs, {}, options);

  opt.run();

  EXPECT_NEAR(0.2, x(0), eps_s);
}

TEST(QP, OneDimTwoConstrained) {
  arr x = arr(1); x(0) = 0;

  arr P(1,1); P(0,0) = 2;
  arr q(1); q(0) = -1;
  arr K(2, 1); K(0, 0) = 1; K(1, 0) = -1;
  arr u(2); u(0) = 0.2; u(1) = 0.0;

  auto qp = std::make_shared<QP_Problem>(P, q, K, u);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp);

  auto options = buildOptions();
  DecOptConstrained<QP_Problem> opt(x, pbs, {}, options);

  opt.run();

  EXPECT_NEAR(0.2, x(0), eps_s);
}

// 2 dims

TEST(QP, TwoDimTwoConstrained) {
  arr x = arr(2);
  x(0) = 0;
  x(1) = 1;

  arr P(2,2);
  P(0,0) = 2;
  P(1,1) = 2;

  arr q(2);
  q(0) = -1;
  q(1) = -1;

  arr K(2, 2);
  K(0, 0) = 1;
  K(1, 1) = -1;

  arr u(2);
  u(0) = 0.2;
  u(1) = -0.8;

  // unconstrained min at (0.5, 0.5)

  auto qp = std::make_shared<QP_Problem>(P, q, K, u);

  std::vector<std::shared_ptr<QP_Problem>> pbs;
  pbs.push_back(qp);

  auto options = buildOptions();
  DecOptConstrained<QP_Problem> opt(x, pbs, {}, options);

  opt.run();

  EXPECT_NEAR(0.2, x(0), eps_s);
  EXPECT_NEAR(0.8, x(1), eps_s);
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
