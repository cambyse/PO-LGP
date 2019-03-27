#include "komo_tree.h"
#include <gtest/gtest.h>
#include <functional>

#include <Core/graph.h>
#include <Core/array.h>

#include <Kin/taskMap_default.h>
#include <Kin/taskMap_transition.h>

#include <tree_builder.h>

using namespace std;
using namespace mp;

//////////////Fixture////////////////
static TreeBuilder build_tree()
{
  auto tb = TreeBuilder();
  tb.add_edge(0, 1); // 0->12
  tb.add_edge(1, 2); // 12->22
  tb.add_edge(2, 3, 0.5); // 22->32
  tb.add_edge(3, 4, 0.5); // 32->42
  tb.add_edge(2, 5, 0.5); // 42->52
  tb.add_edge(5, 6, 0.5); // 52->62
  return tb;
}

struct KomoTreeExtensionFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    kin.init("data/LGP-mono-car.g");
    komo.setModel(kin);

    tb = build_tree();
    n_phases = tb.n_nodes() - 1;

    branch_1 = tb.get_branch(4);
    branch_2 = tb.get_branch(6);
  }

  virtual void TearDown()
  {

  }

  mlr::KinematicWorld kin;
  KOMOTree komo;
  const uint n_micro_steps = 10;
  TreeBuilder tb;
  uint n_phases;
  Branch branch_1;
  Branch branch_2;
};

/////////////////////////////////////

TEST_F(KomoTreeExtensionFixture, TestComputeMicroStepBranch)
{
    // BRANCH 1
    auto u_branch_1 = Branch::computeMicroStepBranch(branch_1, 10);
    auto expected_u_branch_1 = intA(40); // n phase + 2 steps (prefix)
    for(uint t=0; t < 40; t++)
    {
      expected_u_branch_1(t)=t;
    }

    EXPECT_EQ(branch_1.p, u_branch_1.p);
    EXPECT_EQ(expected_u_branch_1, u_branch_1.local_to_global);

    // BRANCH 2
    auto branch_2 = tb.get_branch(6);
    auto u_branch_2 = Branch::computeMicroStepBranch(branch_2, 10);
    auto expected_u_branch_2 = intA(40); // n phase + 2 steps (prefix)
    for(uint t=0; t < 20; t++)
    {
      expected_u_branch_2(t)=t;
    }
    for(uint t=20; t < 40; t++)
    {
      expected_u_branch_2(t)=20+t;
    }

    EXPECT_EQ(branch_2.p, u_branch_2.p);
    EXPECT_EQ(expected_u_branch_2, u_branch_2.local_to_global);
}

TEST_F(KomoTreeExtensionFixture, TestComputeGlobalToBranch)
{
  komo.setTiming( n_phases, n_micro_steps, 1.0, 2 );

  auto acc_1 = komo.setTreeTask(0, -1, branch_1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  auto acc_2 = komo.setTreeTask(0, -1, branch_2, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);

  for(auto t=0; t < komo.T; ++t)
  {
    if( acc_1->prec(t) )
    {
      auto bt = acc_1->branch.global_to_local[t]; // branch time
      auto retrieved_t = acc_1->branch.local_to_global[bt];
      EXPECT_EQ(t, retrieved_t);
    }
  }
}

TEST_F(KomoTreeExtensionFixture, TestBranchPrecSpecificationWithMinusOneEnd)
{
  // prec branch 1
  auto prec_branch_1 = arr(60); // n phase + 2 steps (prefix)
  for( auto s = 0; s < 60; ++s )
  {
    if(s >=0 && s < 40-2)
    {
      prec_branch_1(s)=1;
    }
    else
    {
      prec_branch_1(s)=0;
    }
  }

  // prec branch 2
  auto prec_branch_2 = arr(60); // n phase + 2 steps (prefix)
  for(auto s = 0; s < 60; ++s)
  {
    if( s < 20 || s >= 40 && s < 60-2)
    {
      prec_branch_2(s)=1;
    }
    else
    {
      prec_branch_2(s)=0;
    }
  }
  //

  komo.setTiming( n_phases, n_micro_steps, 1.0, 2 );

  auto acc_1 = komo.setTreeTask(0, -1, branch_1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  auto acc_2 = komo.setTreeTask(0, -1, branch_2, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);

  EXPECT_EQ(Branch::computeMicroStepBranch(branch_1, n_micro_steps), acc_1->branch);
  EXPECT_EQ(prec_branch_1, acc_1->prec);

  EXPECT_EQ(Branch::computeMicroStepBranch(branch_2, n_micro_steps), acc_2->branch);
  EXPECT_EQ(prec_branch_2, acc_2->prec);
}

TEST_F(KomoTreeExtensionFixture, TestBranchPrecSpecificationWithNormalTaskSpecification)
{
  // prec branch 1
  auto prec_branch_1 = arr(60); // n phase * steps_per_phases
  for( auto s = 0; s < 60; ++s )
  {
    if(s >=0 && s < 40-2)
    {
      prec_branch_1(s)=1;
    }
    else
    {
      prec_branch_1(s)=0;
    }
  }

  // prec branch 2
  auto prec_branch_2 = arr(60); // n phase + 2 steps (prefix)
  for(auto s = 0; s < 60; ++s)
  {
    if( s >= 50 && s < 60-2)
    {
      prec_branch_2(s)=1;
    }
    else
    {
      prec_branch_2(s)=0;
    }
  }
  //

  komo.setTiming( n_phases, n_micro_steps, 1.0, 2 );

  arr op_speed_1{ 0.5, 0, 0 };
  arr op_speed_2{ 1.5, 0, 0 };

  auto speed_1 = komo.setTreeTask(0,  4, branch_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  auto speed_2 = komo.setTreeTask(3,  4, branch_2, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);

  EXPECT_EQ(prec_branch_1, speed_1->prec);
  EXPECT_EQ(prec_branch_2, speed_2->prec);
}

TEST_F(KomoTreeExtensionFixture, TestPoseOptimizationOrder1)
{
  komo.setTiming( 1, 2, 1.0, 1 );
  komo.setSquaredQVelocities();

  arr op_speed_1{ 0.5, 0, 0 };

  auto speed_1 = komo.setTask(0,  -1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);

  komo.reset();
  EXPECT_EQ(true, komo.checkGradients());
  komo.run();

  komo.displayTrajectory(0.1, true);
}

TEST_F(KomoTreeExtensionFixture, TestLinearTrajectory)
{
  komo.setTiming( 5, n_micro_steps, 1.0, 2 );

  auto acc_1 = komo.setTask(0, -1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);

  arr op_speed_1{ 0.5, 0, 0 };

  auto speed_1 = komo.setTask(0,  4, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);

  komo.reset();
  EXPECT_EQ(true, komo.checkGradients());
  komo.run();

  komo.displayTrajectory(0.1, true);
}

TEST_F(KomoTreeExtensionFixture, TestSimpleOptimizationWithTwoBranches)
{
  komo.setTiming( n_phases, n_micro_steps, 1.0, 2 );

//  branch_1.p = 0.1;
//  branch_2.p = 0.9;

  auto acc_1 = komo.setTreeTask(0, -1, branch_1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  auto acc_2 = komo.setTreeTask(0, -1, branch_2, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);

  arr op_speed_1{ 0.5, 0, 0 };
  arr op_speed_2{ 1.5, 0, 0 };

  auto speed_1 = komo.setTreeTask(0,  4, branch_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  auto speed_2 = komo.setTreeTask(3,  4, branch_2, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);

  komo.reset();
  EXPECT_EQ(true, komo.checkGradients());
  komo.run();

  komo.displayTrajectory(0.1, true);
  //komo.displayTrajectory(0.1, true);
}

TEST(RowShifting, Understanding_RowShifting)
{
  arr J;
  uint xN = 20;
  uint band_size = 6;// (k+1)*dim_xmax
  uint phiN = xN * 3; // xN * nTask * taskDim
  RowShifted *Jaux = makeRowShifted(J, phiN, band_size, xN);

  EXPECT_EQ(J.d0, phiN);
  EXPECT_EQ(J.d1, band_size);
  EXPECT_EQ(Jaux->real_d1, xN);

  // assign
  J(0, 0) = 1; // store phiN * band_size elements
  J(1, Jaux->rowShift(1)) = 1;

  //J(30, 1) = 1;

  // necessary?
  Jaux->reshift();
  Jaux->computeColPatches(true);

  // unshift
  auto Junshifted = unpack(J); // store phiN * xN

  EXPECT_EQ(Junshifted.d0, phiN);
  EXPECT_EQ(Junshifted.d1, xN);
  EXPECT_EQ(Junshifted(0, 0), 1);
  //EXPECT_EQ(Junshifted(1, 1), 1);
}


////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

