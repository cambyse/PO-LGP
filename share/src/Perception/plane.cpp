#include "plane.h"


CostFct_PlanePoints::CostFct_PlanePoints(const arr& n, const arr& m, const arr& X, const arr& transform)
  : n(n), m(m), X(X), transform(transform){

  arr trans = transform.sub(0,2);
  quat.set(transform.sub(3,6));
  R = quat.getArr();

  y = X*~R*n;
  y += scalarProduct(trans,n);
  y -= scalarProduct(m,n);
}

arr CostFct_PlanePoints::df_transform(){
  arr df_translation = (2. * sum(y)) * ~n;
  arr J;
  tensorPermutation(J, quat.getMatrixJacobian(), TUP(0, 2, 1)); //account for the transpose of R!!
  arr df_quaternion = 2. * (~y * X) * ~(J * n);
  return cat(df_translation, df_quaternion);
}
