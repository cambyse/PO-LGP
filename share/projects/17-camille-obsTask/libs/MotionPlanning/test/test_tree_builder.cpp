#include "komo_planner.h"
#include <gtest/gtest.h>
#include <tree_builder.h>

using namespace std;
using namespace mp;

TreeBuilder build_simple_path_builder()
{
  auto tb = TreeBuilder();
  tb.add_edge(0, 1);
  tb.add_edge(1, 2);
  tb.add_edge(2, 3, 0.5);
  tb.add_edge(2, 4, 0.5);
  return tb;
}

TEST(TreeBuilder, ClassCreation)
{
  auto tb = TreeBuilder();
}

TEST(TreeBuilder, AddEdge)
{
  auto tb = TreeBuilder();
  tb.add_edge(0, 1);

  EXPECT_EQ( tb.n_nodes(), 2 );
  EXPECT_EQ( tb.p(0,1), 1 );

  tb.add_edge(1, 2);

  EXPECT_EQ(tb.n_nodes(), 3);
  EXPECT_EQ(tb.p(0, 1), 1);
  EXPECT_EQ(tb.p(1, 2), 1);

  tb.add_edge(2, 3, 0.5);

  EXPECT_EQ(tb.n_nodes(), 4);
  EXPECT_EQ(tb.p(2, 3), 0.5);

  tb.add_edge(2, 4, 0.5);

  EXPECT_EQ(tb.n_nodes(), 5);
  EXPECT_EQ(tb.p(2, 4), 0.5);
}

TEST(TreeBuilder, GetLeafs)
{
  auto tb =build_simple_path_builder();
  EXPECT_EQ(std::vector<uint>({3, 4}), tb.get_leafs());
}

TEST(TreeBuilder, GetParents)
{
  auto tb = build_simple_path_builder();
  EXPECT_EQ(std::vector<uint>({}), tb.get_parents(0));
  EXPECT_EQ(std::vector<uint>({0}), tb.get_parents(1));
  EXPECT_EQ(std::vector<uint>({1}), tb.get_parents(2));
  EXPECT_EQ(std::vector<uint>({2}), tb.get_parents(3));
  EXPECT_EQ(std::vector<uint>({2}), tb.get_parents(4));
}

TEST(TreeBuilder, GetBranch)
{
  auto tb = build_simple_path_builder();
  Branch expected_branch_to_3;
  expected_branch_to_3.p = 0.5;
  expected_branch_to_3.leaf_id = 3;
  expected_branch_to_3.local_to_global.push_back(0);
  expected_branch_to_3.local_to_global.push_back(1);
  expected_branch_to_3.local_to_global.push_back(2);
  expected_branch_to_3.local_to_global.push_back(3);
  expected_branch_to_3.global_to_local.push_back(0);
  expected_branch_to_3.global_to_local.push_back(1);
  expected_branch_to_3.global_to_local.push_back(2);
  expected_branch_to_3.global_to_local.push_back(3);
  expected_branch_to_3.global_to_local.push_back(-1);
  EXPECT_EQ(expected_branch_to_3, tb.get_branch(3));

  Branch expected_branch_to_4;
  expected_branch_to_4.p = 0.5;
  expected_branch_to_4.leaf_id = 4;
  expected_branch_to_4.local_to_global.push_back(0);
  expected_branch_to_4.local_to_global.push_back(1);
  expected_branch_to_4.local_to_global.push_back(2);
  expected_branch_to_4.local_to_global.push_back(4);
  expected_branch_to_4.global_to_local.push_back(0);
  expected_branch_to_4.global_to_local.push_back(1);
  expected_branch_to_4.global_to_local.push_back(2);
  expected_branch_to_4.global_to_local.push_back(-1);
  expected_branch_to_4.global_to_local.push_back(3);
  EXPECT_EQ(expected_branch_to_4, tb.get_branch(4));
}

TEST(TreeBuilder, GetBranchs)
{
  auto tb = build_simple_path_builder();
  EXPECT_EQ(2, tb.get_branches().size());
}

TEST(TreeBuilder, GetVarsNSteps1)
{
  auto tb = build_simple_path_builder();

  // 0->1
  // order 0
  EXPECT_EQ(intA(1, 1, {0}), tb.get_vars(0, 1.0, 3, 0));

  // order 1
  EXPECT_EQ(intA(1, 2, {-1, 0}), tb.get_vars(0, 1.0, 3, 1));

  // order 2
  EXPECT_EQ(intA(1, 3, {-2, -1, 0}), tb.get_vars(0, 1.0, 3, 2));

  // 1->2
  // order 0
  EXPECT_EQ(intA(1, 1, {1}), tb.get_vars(1.0, 2.0, 3, 0));

  // order 1
  EXPECT_EQ(intA(1, 2, {0, 1}), tb.get_vars(1.0, 2.0, 3, 1));

  // order 2
  EXPECT_EQ(intA(1, 3, {-1, 0, 1}), tb.get_vars(1.0, 2.0, 3, 2));

  // 1->3
  // order 0
  EXPECT_EQ(intA(2, 1, {1, 2}), tb.get_vars(1.0, 3.0, 3, 0));

  // order 1
  EXPECT_EQ(intA(2, 2, {0, 1,  1, 2}), tb.get_vars(1.0, 3.0, 3, 1));

  // order 2
  EXPECT_EQ(intA(2, 3, {-1, 0, 1,  0, 1, 2}), tb.get_vars(1.0, 3.0, 3, 2));

  // 1->4
  // order 0
  EXPECT_EQ(intA(2, 1, {1, 2}), tb.get_vars(1.0, 3.0, 4, 0));

  // order 1
  EXPECT_EQ(intA(2, 2, {0, 1,  1, 2}), tb.get_vars(1.0, 3.0, 4, 1));

  // order 2
  EXPECT_EQ(intA(2, 3, {-1, 0, 1,  0, 1, 2}), tb.get_vars(1.0, 3.0, 4, 2));
}

TEST(TreeBuilder, GetVarsNSteps2)
{
  auto tb = build_simple_path_builder();
  uint steps = 2;
  // 0->1
  // order 0
  EXPECT_EQ(intA(2, 1, {0, 1}), tb.get_vars(0, 1.0, 3, 0, steps));

  // order 1
  EXPECT_EQ(intA(2, 2, {-1, 0,  0, 1}), tb.get_vars(0, 1.0, 3, 1, steps));

  // order 2
  EXPECT_EQ(intA(2, 3, {-2, -1, 0,  -1, 0, 1}), tb.get_vars(0, 1.0, 3, 2, steps));

  // 1->3
  // order 0
  EXPECT_EQ(intA(4, 1, {2, 3, 4, 5}), tb.get_vars(1.0, 3.0, 3, 0, steps));

  // order 1
  EXPECT_EQ(intA(4, 2, {1, 2,  2, 3,  3, 4,  4, 5}), tb.get_vars(1.0, 3.0, 3, 1, steps));

  // order 2
  EXPECT_EQ(intA(4, 3, {0, 1, 2,  1, 2, 3,  2, 3, 4,  3, 4, 5}), tb.get_vars(1.0, 3.0, 3, 2, steps));

  // 1->4
  // order 0
  EXPECT_EQ(intA(4, 1, {2, 3, 6, 7}), tb.get_vars(1.0, 3.0, 4, 0, steps));
}

TEST(TreeBuilder, GetVarsNSteps10)
{
  auto steps = 10;

  TreeBuilder tb;
  tb.add_edge(0, 1);
  tb.add_edge(1, 2);
  tb.add_edge(1, 3);
  // order 1
  EXPECT_EQ(intA(10, 1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}), tb.get_vars(0, 1.0, 2, 0, steps));
  EXPECT_EQ(intA(10, 1, {20, 21, 22, 23, 24, 25, 26, 27, 28, 29}), tb.get_vars(1.0, 2.0, 3, 0, steps));
  EXPECT_EQ(intA(10, 2, {9, 10,  10, 11,  11, 12,  12, 13,  13, 14,  14, 15,  15, 16,  16, 17,  17, 18,  18, 19}), tb.get_vars(1.0, 2.0, 2 , 1, steps));
  EXPECT_EQ(intA(10, 2, {9, 20,  20, 21,  21, 22,  22, 23,  23, 24,  24, 25,  25, 26,  26, 27,  27, 28,  28, 29}), tb.get_vars(1.0, 2.0, 3, 1, steps));
}

TEST(TreeBuilder, GetVarsNSteps5)
{
  auto steps = 5;

  TreeBuilder tb;
  tb.add_edge(0, 1);

  tb.add_edge(1, 2);
  tb.add_edge(2, 3);

  tb.add_edge(1, 4);
  tb.add_edge(4, 5);

  auto leafs = tb.get_leafs();
  // order 0
  EXPECT_EQ(intA(15, 1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}), tb.get_vars(0, 3.0, 3, 0, steps));
  EXPECT_EQ(intA(15, 1, {0, 1, 2, 3, 4, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24}), tb.get_vars(0, 3.0, 5, 0, steps));
}


////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

