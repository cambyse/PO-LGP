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

  void f(const arr &x, arr &y);
  void dfdmu(const arr &x, arr &g, arr& H=NoArr);
  void dfdstd(const arr &x, arr &g, arr& H=NoArr);
  void dfdw(const arr &x, arr &g, arr& H=NoArr);


  void dfdwdmu(const arr &x,arr& H);
  void dfdwdstd(const arr &x,arr& H);
  void dfdmudstd(const arr &x,arr& H);
};

#endif // GAUSSIAN_COSTS_H
