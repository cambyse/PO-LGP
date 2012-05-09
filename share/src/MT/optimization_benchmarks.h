#include "optimization.h"

//===========================================================================

/// $f(x) = x^T C x$ where C has eigen values ranging from 1 to 'condition'
struct SquaredCost:public ScalarFunction,VectorFunction {
  arr M,C; /// $C = M^T M $
  uint n;  /// dimensionality of $x$
  
  SquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);
  
  double fs(arr& grad,const arr& x);
  void fv(arr& y, arr& J,const arr& x);
};

//===========================================================================

/// Same as SquaredCost but $x_i \gets atan(x_i)$ before evaluating the squared cost
struct NonlinearlyWarpedSquaredCost:public ScalarFunction,VectorFunction {
  uint n;  /// dimensionality of $x$
  SquaredCost sq;
  
  NonlinearlyWarpedSquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);
  
  double fs(arr& grad,const arr& x);
  void fv(arr& y, arr& J,const arr& x);
};

//===========================================================================

struct VectorChainCost:VectorChainFunction {
  uint n;
  arr A,a;
  arr Wi,Wj,w;
  bool nonlinear;
  
  VectorChainCost(uint _T,uint _n);
  void fvi(arr& y, arr* J, uint i, const arr& x_i);
  void fvij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j);
};

//===========================================================================

struct SlalomProblem:VectorChainFunction {
  uint K,n;
  double margin,w,power;
  
  SlalomProblem(uint _T, uint _K, double _margin, double _w, double _power);
  void fvi(arr& y, arr& J, uint i, const arr& x_i);
  void fvij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j);
};
