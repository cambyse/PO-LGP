#include "plane.h"


CostFct_PlanePoints::CostFct_PlanePoints(const arr& n, const arr& m, const arr& X, const arr& transform)
  : n(n), m(m), X(X), transform(transform), r(transform.sub(3,6)){
  y = X*~r.getArr()*n;
  y += scalarProduct(transform.sub(0,2),n);
  y -= scalarProduct(m,n);
}

double CostFct_PlanePoints::f(){ return sumOfSqr(y); }

arr CostFct_PlanePoints::df_transform(){
  arr df_translation = (2. * sum(y)) * ~n;
  arr J;
  tensorPermutation(J, r.getMatrixJacobian(), TUP(0, 2, 1)); //account for the transpose of R!!
  arr df_quaternion = 2. * (~y * X) * ~(J * n);
  return cat(df_translation, df_quaternion);
}

ScalarFunction CostFct_PlanePoints::f_transform(){
  return [this](arr& g, arr& H, const arr& x) -> double {
    CostFct_PlanePoints fx(n,m,X,x);
    if(&g) g=fx.df_transform();
    return fx.f();
  };
}
