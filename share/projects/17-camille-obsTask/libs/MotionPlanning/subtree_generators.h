#pragma once

#include <Core/array.h>
#include <tree_builder.h>
#include <stdlib.h>
#include <ostream>
#include <list>

namespace mp
{

struct SubTreeGen
{
  virtual bool finished() const = 0;
  virtual TreeBuilder next() = 0;
};

struct BranchGen : public SubTreeGen
{
  BranchGen(const TreeBuilder& tree)
    : tree(tree)
  {
    leaves = tree.get_leaves();
  }

  bool finished() const override
  {
    return (index == leaves.size());
  }

  TreeBuilder next() override
  {
    CHECK(!finished(), "finished generator");

    return tree.get_branch(leaves[index++]);
  }

  const TreeBuilder& tree;
  std::vector<uint> leaves;
  uint index = 0;
};

struct SubTreesAfterFirstBranching : public SubTreeGen // common trunk
{
  SubTreesAfterFirstBranching(const TreeBuilder& tree);

  bool finished() const override;

  TreeBuilder next() override;

  const TreeBuilder& tree;
  std::vector<uint> path_to_source;
  std::vector<uint> sources;
  uint index = 0;
};

struct LinearSplit : public SubTreeGen // common trunk
{
  LinearSplit(const TreeBuilder& tree, uint n);

  bool finished() const override;

  TreeBuilder next() override;

  const TreeBuilder& tree;
  uint n;
  std::vector<std::pair<uint, uint>> splits;
  uint index{0};
};

struct GeneratorFactory
{
  /**
   * @brief create generator for decomposing the tree
   * @param name
   * @param n ideal number of subtrees
   * @param tree
   * @return
   */
  std::shared_ptr<SubTreeGen> create(const std::string& name, uint n, const TreeBuilder& tree) const
  {
    if(name == "BranchGen")
    {
      return std::make_shared<BranchGen>(tree);
    }
    else if(name == "SubTreesAfterFirstBranching")
    {
      return std::make_shared<SubTreesAfterFirstBranching>(tree);
    }
    else if(name == "LinearSplit")
    {
      return std::make_shared<LinearSplit>(tree, n);
    }
  }
};

}

