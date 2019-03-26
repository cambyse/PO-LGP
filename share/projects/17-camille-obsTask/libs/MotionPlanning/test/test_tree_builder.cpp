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

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

