#pragma once

#include <stdlib.h>
#include <Core/array.h>

#include "komo_tree.h"

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
    intA get_vars(double from, double to, uint leaf, uint order=2, uint steps=1) const;
    //std::vector<intA> get_all_vars(uint order=2, uint steps=1) const;
    arr get_scales(double from, double to, uint leaf, uint steps=1) const;

    void add_edge(uint from, uint to, double p = 1.0);

   private:
    arr adjacency_matrix_;
  };
}
