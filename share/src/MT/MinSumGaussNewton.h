#ifndef MT_MinSumGaussNewton_h
#define MT_MinSumGaussNewton_h

#include <fstream>
#include "array.h"

struct Fij { arr A, B, C, a, b; double hata; };
struct Mu { arr M, m;       double hatm; };

struct MinSumGaussNewton {
  arr x;
  double tolerance, maxStep;
  
  uintA Msgs; //edges: nx2 array for n edges
  /* Actually: this is not the edge set but rather the set of message indeces:
     it includes forward and backward tuples (i,j) and (j,i) and also (i,i) to index node potentials
     (message mu_{i\to i})
     
     Both arrays 'fij' and 'mu' below are indexed this way. This is clear for mu.
     For fij: we compute the pair-wise potential fij twice: once for usage
     in the backward message (update equation) and once for usage in the forward update equation.
     Why this redundant computation and storage? Because it simplifies the code.
     */
  MT::Array<uintA> del; //in-neighbors
  MT::Array<Fij> fij;
  MT::Array<Mu>  mu;
  boolA clamped;
  std::ofstream fil;

  void setUndirectedGraph(uint n,const uintA& E);
  
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
