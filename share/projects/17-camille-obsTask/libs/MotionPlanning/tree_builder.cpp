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

  intA TreeBuilder::get_vars0(double from, double to, uint leaf, uint steps) const
  {
    auto branch = get_branch(leaf);
    const auto duration = ceil(to - from);
    uint d0 = duration * steps;
    uint from_step = from * steps;

    intA vars(d0);

    for(auto t=0; t < d0; ++t)
    {
      int k = from_step + t;

      if(k < 0) // prefix handling (we don't branch during the prefix)
      {
        vars(t) = k;
      }
      else
      {
        int from_node = branch.local_to_global[floor(k / double(steps))];
        int to_node   = branch.local_to_global[ceil(k / double(steps) + 0.00001)];
        int r = k % steps;

        vars(t) = (steps > 1 ? to_node - 1 : from_node) * steps + r; // in new branched phase
      }
    }

    return vars;
  }

  intA TreeBuilder::get_vars(double from, double to, uint leaf, uint order, uint steps) const
  {
    std::vector<intA> splitted_vars(order+1);// = get_vars0(from, to, leaf, steps);
    for(auto j = 0; j < order+1; ++j)
    {
      auto delta = double(j) / steps;
      splitted_vars[j] = get_vars0(from - delta, to - delta, leaf, steps);
    }

    auto d0 = splitted_vars.front().d0;
    intA vars(d0, order + 1);
    for(auto i = 0; i < d0; ++i)
    {
      for(auto j = 0; j < order+1; ++j)
      {
        vars(i, order - j) = splitted_vars[j](i);
      }
    }

    return vars;
  }

  arr TreeBuilder::get_scales(double from, double to, uint leaf, uint steps) const
  {
      auto branch = get_branch(leaf);
      const auto duration = to - from;
      uint d0 = duration * steps;
      uint from_step = from * steps;

      //arr scales(d0);
      arr full_scale = arr((branch.local_to_global.size() - 1) * steps);

      double p = 1.0;
      for(auto i = 0; i < branch.local_to_global.size() - 1; ++i)
      {
          auto global_i = branch.local_to_global[i];
          auto global_j = branch.local_to_global[i+1];
          p *= adjacency_matrix_(global_i, global_j);

          for(auto s = 0; s < steps; ++s)
          {
              full_scale(steps * i + s) = p;
          }
      }

      arr scale(d0);
      for(auto i = 0; i < d0; ++i)
      {
          scale(i) = full_scale(i + from_step);
      }

      return scale;
  }

  void TreeBuilder::add_edge(uint from, uint to, double p)
  {
    uint max = std::max(from, to);
    auto size = max+1;

    if(adjacency_matrix_.d0 < size)
    {
      auto old_adjacency_matrix = adjacency_matrix_;
      const auto& old_size = old_adjacency_matrix.d0;
      auto adjacency_matrix = arr(size, size);
      adjacency_matrix.setMatrixBlock(old_adjacency_matrix, 0, 0);
      adjacency_matrix_ = adjacency_matrix;
    }

    adjacency_matrix_(from, to) = p;
  }
}
