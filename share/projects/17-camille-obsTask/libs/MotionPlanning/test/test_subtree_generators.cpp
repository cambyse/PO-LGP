#include "komo_planner.h"
#include <gtest/gtest.h>
#include <subtree_generators.h>
#include <test/trees.h>

using namespace std;
using namespace mp;

TEST(TreeBuilder, BranchGenerator)
{
  auto tree = build_simple_path_builder();

  auto gen = BranchGen(tree);

  auto b1 = gen.next();
  auto b2 = gen.next();

  EXPECT_EQ(0.5, b1.p());
  EXPECT_EQ(0.5, b2.p());

  EXPECT_EQ(std::vector<uint>({0, 1, 2, 3}), b1.get_nodes());
  EXPECT_EQ(std::vector<uint>({0, 1, 2, 4}), b2.get_nodes());

  EXPECT_TRUE(gen.finished());
}

TEST(TreeBuilder, SubTreesAfterFirstBranching)
{
  auto tree = build_5_edges_2_branchings();

  auto gen = SubTreesAfterFirstBranching(tree);

  auto s1 = gen.next();
  auto s2 = gen.next();

  EXPECT_EQ(0.4, s1.p());
  EXPECT_EQ(0.6, s2.p());

  EXPECT_EQ(std::vector<uint>({0, 1, 2}), s1.get_nodes());
  EXPECT_EQ(std::vector<uint>({0, 1, 3, 4, 5}), s2.get_nodes());

  EXPECT_TRUE(gen.finished());
}

TEST(TreeBuilder, SubTreesAfterFirstBranchingSpecs)
{
  auto steps = 5;

  auto tree = build_5_edges_2_branchings();

  auto gen = SubTreesAfterFirstBranching(tree);

  auto s1 = gen.next();
  Mapping m1;
  auto s1c = s1.compressed(m1);

  auto s2 = gen.next();
  Mapping m2;
  auto s2c = s2.compressed(m2);

  auto spec = s2c.get_spec({0.0, -1.0}, Edge{0, 1}, 1, 5);

  std::cout << s1c.adjacency_matrix() << std::endl;
  std::cout << s2c.adjacency_matrix() << std::endl;
}

TEST(TreeBuilder, LinearSplit)
{
  auto steps = 5;
  auto tree = build_2_linear_edges();

  auto gen = LinearSplit(tree, 8);

  auto s1 = gen.next().get_nodes();
  auto s2 = gen.next().get_nodes();

  EXPECT_EQ(std::vector<uint>({0, 1}), s1);
  EXPECT_EQ(std::vector<uint>({1, 2}), s2);

  EXPECT_TRUE(gen.finished());
}

TEST(InteractingEdges, LinearFullTree)
{
  auto steps = 5;
  auto tree = build_2_linear_edges();
  auto sub = build_2_linear_edges_sub_1();

  auto edges = interactingEdges(tree, sub);

  EXPECT_EQ(Edge({0, 1}), edges[0]);
  EXPECT_EQ(Edge({1, 2}), edges[1]);
}

TEST(InteractingEdges, TreeSub1)
{
  auto steps = 5;
  auto tree = build_3_edges_1_branching();
  auto sub = build_3_edges_1_branching_sub_1();

  auto edges = interactingEdges(tree, sub);

  EXPECT_EQ(Edge({0, 1}), edges[0]);
  EXPECT_EQ(Edge({1, 2}), edges[1]);
}

TEST(InteractingEdges, TreeSub2)
{
  auto steps = 5;
  auto tree = build_3_edges_1_branching();
  auto sub = build_3_edges_1_branching_sub_2();

  auto edges = interactingEdges(tree, sub);

  EXPECT_EQ(Edge({0, 1}), edges[0]);
  EXPECT_EQ(Edge({1, 3}), edges[1]);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

