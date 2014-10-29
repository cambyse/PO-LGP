#ifndef GAUSSIAN_COSTS_H
#define GAUSSIAN_COSTS_H

#include <Core/array.h>
#include "gaussian_costs.h"



struct GaussianCosts {

  double mu;
  double std;
  double w;

  GaussianCosts() {
  };

  void f(arr &x, arr &y);
  void dfdmu(arr &x, arr &g, arr& H=NoArr);
  void dfdstd(arr &x, arr &g, arr& H=NoArr);
  void dfdw(arr &x, arr &g, arr& H=NoArr);
};

#endif // GAUSSIAN_COSTS_H
