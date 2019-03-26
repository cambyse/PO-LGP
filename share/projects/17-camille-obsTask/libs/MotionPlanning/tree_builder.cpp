#include <tree_builder.h>

namespace mp
{
  TreeBuilder::TreeBuilder()
    : adjacency_matrix_ ( arr(uint(0), uint(0)) )
  {

  }

  uint TreeBuilder::n_nodes() const
  {
    return adjacency_matrix_.d0;
  }

  double TreeBuilder::p(uint from, uint to) const
  {
    return adjacency_matrix_(from, to);
  }

  std::vector<uint> TreeBuilder::get_leafs() const
  {
    std::vector<uint> leafs;

    auto all_zeros = [](const arr & row)
    {
      CHECK_EQ(row.d0, 1, "wrong row dimensions");

      for(auto j = 0; j < row.d1; ++j)
      {
        if(row(0, j) != 0)
        {
          return false;
        }
      }

      return true;
    };

    for(auto i = 0; i < adjacency_matrix_.d0; ++i)
    {
      if(all_zeros(adjacency_matrix_.row(i)))
      {
        leafs.push_back(i);
      }
    }

    return leafs;
  }

  std::vector<uint> TreeBuilder::get_parents(uint node) const
  {
    std::vector<uint> parents;

    auto col = adjacency_matrix_.col(node);

    for(uint i = 0; i < n_nodes(); ++i)
    {
      if(col(i, 0) != 0)
      {
        parents.push_back(i);
      }
    }

    return parents;
  }

  Branch TreeBuilder::get_branch(uint leaf) const
  {
    Branch branch;
    auto current = leaf;
    auto parents = get_parents(current);

    CHECK(parents.size() > 0, "No parents for this leaf!");
    CHECK(parents.size() < 2, "Not implemented yet!, needs graph support!");

    branch.p = p(parents[0], leaf);
    branch.leaf_id = leaf;

    while(parents.size())
    {
      auto parent = parents[0];
      auto p = this->p(parent, current);
      branch.local_to_global.push_back(current);
      current = parent;
      parents = get_parents(current);
    }

    branch.local_to_global.push_back(0);
    std::reverse(branch.local_to_global.begin(), branch.local_to_global.end());

    branch.global_to_local = std::vector< int >(n_nodes(), -1);
    for( auto local = 0; local < branch.local_to_global.size(); ++local )
    {
      auto global = branch.local_to_global[local];
      branch.global_to_local[global] = local;
    }

    return branch;
  }

  std::vector<Branch> TreeBuilder::get_branches() const
  {
    std::vector<Branch> branches;

    for(auto l : get_leafs())
    {
      branches.push_back(get_branch(l));
    }

    return branches;
  }

  void TreeBuilder::add_edge(uint from, uint to, double p)
  {
    uint max = std::max(from, to);
    auto size = max+1;

    if(adjacency_matrix_.d0 < size)
    {
      auto old_adjacency_matrix = adjacency_matrix_;
      auto old_size = old_adjacency_matrix.d0;
      auto adjacency_matrix = arr(size, size);
      adjacency_matrix.setMatrixBlock(old_adjacency_matrix, 0, 0);
      adjacency_matrix_ = adjacency_matrix;
    }

    adjacency_matrix_(from, to) = p;
  }
}
