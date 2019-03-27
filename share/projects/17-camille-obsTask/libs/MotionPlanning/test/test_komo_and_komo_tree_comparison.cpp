#include "komo_tree.h"
#include <gtest/gtest.h>
#include <functional>
#include <list>
#include <Core/graph.h>
#include <Core/array.h>

#include <Kin/taskMap_default.h>
#include <Kin/taskMap_transition.h>

#include <tree_builder.h>

using namespace std;
using namespace mp;

/*
 *
 * Compare performances of KOMO and Tree KOMO
 *
 */

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

struct KomoTreeComparisonFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    kin.init("data/LGP-mono-car.g");
  }

  virtual void TearDown()
  {

  }

  mlr::KinematicWorld kin;
};
///////TREE//////////////
/*
 *  Type 1:
 *  |
 * / \
 *
 */

double plan_tree_traj_type_1_with_komo_tree(uint micro_steps, const mlr::KinematicWorld & kin)
{
  TreeBuilder tb;
  tb.add_edge(0, 1, 1.0);
  tb.add_edge(1, 2, 0.5);
  tb.add_edge(1, 3, 0.5);

  auto branch_1 = tb.get_branch(2);
  auto branch_2 = tb.get_branch(3);
  uint n_phases = tb.n_nodes() - 1;

  KOMOTree komo; komo.setModel(kin);

  komo.setTiming( n_phases, micro_steps, 1.0, 2 );

  arr op_speed_1{ 0.5, 0, 0 };
  arr op_speed_2{ 1.5, 0, 0 };

  komo.setTreeTask(0, -1, branch_1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  komo.setTreeTask(0, -1, branch_2, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);

  komo.setTreeTask(0,  1, branch_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(1,  2, branch_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(1,  2, branch_2, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);

  komo.reset();
  //EXPECT_EQ(true, komo.checkGradients());
  mlr::timerStart();
  komo.run();
  auto time = mlr::timerPause();

  //komo.displayTrajectory(0.1, true);

  return time;
}

double plan_tree_traj_type_1_with_komo(uint micro_steps, const mlr::KinematicWorld & kin)
{
  double time = 0;
  // plan for branch 1
  {
    KOMO komo; komo.setModel(kin);
    komo.setTiming( 2, micro_steps, 1.0, 2 );

    arr op_speed_1{ 0.5, 0, 0 };

    komo.setTask(0, -1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
    komo.setTask(0,  1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);

    komo.reset();

    //EXPECT_EQ(true, komo.checkGradients());
    mlr::timerStart();
    komo.run();
    time += mlr::timerPause();
  }
  // plan for branch 2
  {
    KOMO komo; komo.setModel(kin);
    komo.setTiming( 2, micro_steps, 1.0, 2 );

    arr op_speed_2{ 1.5, 0, 0 };
    komo.setTask(0, -1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
    komo.setTask(0,  1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);

    komo.reset();

    //EXPECT_EQ(true, komo.checkGradients());
    mlr::timerStart();
    komo.run();
    time += mlr::timerPause();
  }

  return 2 * time; // simulate other pass of optimization with additional equality constraint
}

TEST_F(KomoTreeComparisonFixture, TestKOMOTreeOnTree1Example)
{
  uint micro_steps = 20;
  double time = 0;
  EXPECT_NO_THROW( time = plan_tree_traj_type_1_with_komo_tree(micro_steps, kin) );

  std::cout << "TestKOMOTreeOnTree1Example:"<< time << std::endl;
}

TEST_F(KomoTreeComparisonFixture, TestKOMOOnTree1Example)
{
  uint micro_steps = 20;
  double time = 0;
  EXPECT_NO_THROW( time = plan_tree_traj_type_1_with_komo(micro_steps, kin) );

  std::cout << "TestKOMOOnTree1Example:"<< time << std::endl;
}

/*
 *  Type 2:
 *
 *     |
 *    / \
 *   /\ /\
 */

double plan_tree_traj_type_2_with_komo_tree(uint micro_steps, const mlr::KinematicWorld & kin)
{
  TreeBuilder tb;
  tb.add_edge(0, 1, 1.0);
  tb.add_edge(1, 2, 0.5);
  tb.add_edge(2, 3, 0.25);
  tb.add_edge(2, 4, 0.25);

  tb.add_edge(1, 5, 0.5);
  tb.add_edge(5, 6, 0.5);
  tb.add_edge(5, 7, 0.25);

  auto branch_1 = tb.get_branch(3);
  auto branch_2 = tb.get_branch(4);
  auto branch_3 = tb.get_branch(6);
  auto branch_4 = tb.get_branch(7);
  uint n_phases = tb.n_nodes() - 1;

  KOMOTree komo; komo.setModel(kin);

  komo.setTiming( n_phases, micro_steps, 1.0, 2 );

  arr op_speed_1{ 0.5, 0, 0 };
  arr op_speed_2{ 1.0, 0, 0 };
  arr op_speed_3{ 1.5, 0, 0 };
  arr op_speed_4{ 2.0, 0, 0 };

  komo.setTreeTask(0, -1, branch_1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  komo.setTreeTask(0, -1, branch_2, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  komo.setTreeTask(0, -1, branch_3, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  komo.setTreeTask(0, -1, branch_4, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);

  komo.setTreeTask(0,  1, branch_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(0,  1, branch_2, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(1,  2, branch_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(1,  2, branch_2, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);

  komo.setTreeTask(0,  1, branch_3, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(0,  1, branch_4, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(1,  2, branch_3, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);
  komo.setTreeTask(1,  2, branch_4, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);

  komo.setTreeTask(2,  3, branch_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(2,  3, branch_2, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);
  komo.setTreeTask(2,  3, branch_1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);
  komo.setTreeTask(2,  3, branch_2, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_2, 1.0, 1);

  komo.setTreeTask(2,  3, branch_3, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_3, 1.0, 1);
  komo.setTreeTask(2,  3, branch_4, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_4, 1.0, 1);
  komo.setTreeTask(2,  3, branch_3, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_3, 1.0, 1);
  komo.setTreeTask(2,  3, branch_4, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_4, 1.0, 1);

  komo.reset();
  //EXPECT_EQ(true, komo.checkGradients());
  mlr::timerStart();
  komo.run();
  auto time = mlr::timerPause();

  komo.displayTrajectory(0.1, true);

  return time;
}

TEST_F(KomoTreeComparisonFixture, TestKOMOTreeOnTree2Example)
{
  uint micro_steps = 10;
  double time = 0;
  EXPECT_NO_THROW( time = plan_tree_traj_type_2_with_komo_tree(micro_steps, kin) );

  std::cout << "TestKOMOTreeOnTree2Example:"<< time << std::endl;
}

/////////////TRAJ//////////////////
double plan_linear_traj(uint n_phases, uint micro_steps, KOMO & komo)
{
  const arr op_speed_1{ 0.5, 0, 0 };

  // komo
  komo.setTiming( n_phases, micro_steps, 1.0, 2 );
  komo.setTask(0, -1, new TaskMap_Transition(komo.world), OT_sumOfSqr, NoArr, 1.0, 2);
  komo.setTask(0, -1, new TaskMap_Default(posTMT, komo.world, "car_ego", NoVector, NULL, NoVector), OT_sumOfSqr, op_speed_1, 1.0, 1);

  komo.reset();
//  EXPECT_EQ(true, komo.checkGradients());

  mlr::timerStart();
  komo.run();
  return mlr::timerPause();
}

TEST_F(KomoTreeComparisonFixture, TestKOMOOnLinearTrajectory)
{
  KOMO komo; komo.setModel(kin);
  EXPECT_NO_THROW( plan_linear_traj(20, 10, komo) );
}

TEST_F(KomoTreeComparisonFixture, TestKOMOTreeOnLinearTrajectory)
{
  KOMOTree komo_tree; komo_tree.setModel(kin);
  EXPECT_NO_THROW( plan_linear_traj(20, 10, komo_tree) );
}

TEST_F(KomoTreeComparisonFixture, TestCompareOnLinearTrajectory)
{
  using TestConfig = std::pair< uint, uint >;
  using TimingResult = std::pair< double, double >;
  std::list< TestConfig > test_configs;
  std::list< std::pair< TestConfig, TimingResult > > timing_results;

  test_configs.push_back(std::make_pair(1, 5));
  test_configs.push_back(std::make_pair(1, 10));
  test_configs.push_back(std::make_pair(1, 20));

  test_configs.push_back(std::make_pair(5, 1));
  test_configs.push_back(std::make_pair(5, 5));
  test_configs.push_back(std::make_pair(5, 10));
  test_configs.push_back(std::make_pair(5, 20));

  test_configs.push_back(std::make_pair(10, 1));
  test_configs.push_back(std::make_pair(10, 5));
  test_configs.push_back(std::make_pair(10, 10));
  test_configs.push_back(std::make_pair(10, 20));

//  test_configs.push_back(std::make_pair(20, 1));
//  test_configs.push_back(std::make_pair(20, 5));
//  test_configs.push_back(std::make_pair(20, 10));
//  test_configs.push_back(std::make_pair(20, 20));

//  test_configs.push_back(std::make_pair(40, 1));
//  test_configs.push_back(std::make_pair(40, 5));
//  test_configs.push_back(std::make_pair(40, 10));
//  test_configs.push_back(std::make_pair(40, 20));

  for( auto test_config : test_configs )
  {
    KOMO komo; komo.setModel(kin);
    auto t1 = plan_linear_traj(test_config.first, test_config.second, komo);

    KOMOTree komo_tree; komo_tree.setModel(kin);
    auto t2 = plan_linear_traj(test_config.first, test_config.second, komo_tree);

    timing_results.push_back(std::make_pair(test_config, std::make_pair(t1, t2)));
  }

  for( auto t : timing_results )
  {
    auto test_config = t.first;
    auto timing = t.second;

    std::cout << "n_phases:" << test_config.first << " micro_steps:" << test_config.second << " x:" << test_config.first * test_config.second << " komo:" << timing.first << " komo tree:" << timing.second << std::endl;
  }
  //komo.displayTrajectory(0.1, true);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

