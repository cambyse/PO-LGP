#include <task_tree_builder.h>

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

  std::vector<std::pair<uint, double>> TreeBuilder::get_branch(uint leaf) const
  {
    std::vector<std::pair<uint, double>> branch;
    auto current = leaf;
    auto parents = get_parents(current);

    while(parents.size()==1)
    {
      auto parent = parents[0];
      auto p = this->p(parent, current);
      branch.push_back(std::make_pair(current, p));
      current = parent;
      parents = get_parents(current);
    }

    CHECK(parents.size() < 2, "Not implemented yet!, needs graph support");

    branch.push_back(std::make_pair(0, 1.0));
    std::reverse(branch.begin(), branch.end());
    return branch;
  }

  std::vector<std::vector<std::pair<uint, double>>> TreeBuilder::get_branches() const
  {
    std::vector<std::vector<std::pair<uint, double>>> branches;

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
