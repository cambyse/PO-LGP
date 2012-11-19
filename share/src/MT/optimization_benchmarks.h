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
  uint T,n;
  arr A,a;
  arr Wi,Wj,w;
  bool nonlinear;
  
  VectorChainCost(uint _T,uint _n);
  uint get_T(){ return T; }
  void fv_i(arr& y, arr* J, uint i, const arr& x_i);
  void fv_ij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j);
};

//===========================================================================

struct SlalomProblem:VectorChainFunction {
  uint T,K,n;
  double margin,w,power;
  
  SlalomProblem(uint _T, uint _K, double _margin, double _w, double _power);
  uint get_T(){ return T; }
  void fv_i(arr& y, arr& J, uint i, const arr& x_i);
  void fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j);
};
