#pragma once

#include <stdlib.h>
#include <Core/array.h>

#include "komo_tree.h"

namespace mp
{
struct Vars // Branch
{
  intA order0;
  intA order1;
  intA order2;
  uint microSteps;
  const uint k_order = 2;

  Vars(const intA & order0, const intA& order1, const intA& order2, uint microSteps)
    : order0(order0)
    , order1(order1)
    , order2(order2)
    , microSteps(microSteps)
  {

  }

  intA getVars(double from, double to, uint order) const
  {
    CHECK(from>=0.0, "invalid start time");
    CHECK(to<order0.d0*microSteps, "invalid end time");

    uint indexFrom = from * microSteps;
    uint indexTo = to > 0 ? to * microSteps : (*this)[order].d0;

    if(indexFrom > 1) indexFrom--;
    indexTo--;

    CHECK(indexFrom>=0, "invalid start index time");
    CHECK(indexTo<order0.d0, "invalid end index time");

    return (*this)[order].sub(indexFrom, indexTo, 0, -1);
  }

  int getStep(double time) const
  {
    //int step = (floor(time*double(microSteps) + .500001))-1;
    int step = time * microSteps - 1;
    return order0(step, 0);
  }

  const intA& operator[](std::size_t i) const
  {
    CHECK(i <= 2, "wrong order request!");
    switch(i)
    {
      case 0:
        return order0;
      case 1:
        return order1;
      case 2:
        return order2;
      default:
        break;
    }
  }
};

class TreeBuilder
{
public:
  TreeBuilder();

  uint n_nodes() const;
  double p(uint from, uint to) const;
  std::vector<uint> get_leafs() const;
  std::vector<uint> get_parents(uint node) const;
  _Branch get_branch(uint leaf) const;
  std::vector<_Branch> get_branches() const;
  intA get_vars0(double from, double to, uint leaf, uint steps=1) const;
  intA get_vars(double from, double to, uint leaf, uint order=2, uint steps=1) const;
  arr get_scales(double from, double to, uint leaf, uint steps=1) const;

  void add_edge(uint from, uint to, double p = 1.0);

private:
  arr adjacency_matrix_;
};
}
