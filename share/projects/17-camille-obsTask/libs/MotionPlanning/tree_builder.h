#pragma once

#include <stdlib.h>
#include <Core/array.h>

#include <KOMO/task.h>

namespace mp
{
  class TreeBuilder
  {
  public:
    TreeBuilder();

    uint n_nodes() const;
    double p(uint from, uint to) const;
    std::vector<uint> get_leafs() const;
    std::vector<uint> get_parents(uint node) const;
    Branch get_branch(uint leaf) const;
    std::vector<Branch> get_branches() const;

    void add_edge(uint from, uint to, double p = 1.0);

   private:
    arr adjacency_matrix_;
  };
}
