#ifndef MT_MinSumGaussNewton_h
#define MT_MinSumGaussNewton_h

#include "array.h"

struct Fij { arr A, B, C, a, b; double hata; };
struct Mu { arr M, m;       double hatm; };

struct MinSumGaussNewton {
  arr x;
  double tolerance, maxStep;
  
  uintA E; //edges
  MT::Array<uintA> del; //in-neighbors
  MT::Array<Fij> fij;
  MT::Array<Mu>  mu;
  boolA clamped;
  ofstream fil;
  
  //indirect GaussNewton type problem interface:
  virtual void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){ throw("NIY"); }
  //direct factor type proble interface:
  virtual double f(uint i, uint j, const arr& x_i, const arr& x_j);
  virtual void reapproxPotentials(uint i, const arr& hat_x_i);
  
  void updateMessage(uint m);
  void updateMessagesToNode(uint i);
  double totalCost(bool verbose=false);
  void init();
  void step(uint steps);
};

#ifdef  MT_IMPLEMENTATION
#  include "MinSumGaussNewton.cpp"
#endif

#endif
