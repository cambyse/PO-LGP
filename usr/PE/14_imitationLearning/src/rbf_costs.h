#ifndef RBF_COSTS_H
#define RBF_COSTS_H

#include <Core/array.h>

struct RBFCosts {

  uint nB; // number of basis functions

  arr M;  // mean values
  arr C;  // std values
  arr W;  // weight values of basis functions

  RBFCosts() {
  };

  void f(const arr &x, arr &y);
  void dfdM(const arr &x, arr &g, arr& H=NoArr);

//  void dfdmu(const arr &x, arr &g, arr& H=NoArr);
//  void dfdstd(const arr &x, arr &g, arr& H=NoArr);



//  void dfdwdmu(const arr &x,arr& H);
//  void dfdwdstd(const arr &x,arr& H);
//  void dfdmudstd(const arr &x,arr& H);
};

#endif // RBF_COSTS_H
