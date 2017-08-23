#include "taskMap_linTrans.h"

void TaskMap_LinTrans::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map->phi(y, J, G, t);
  if(!norm){
    if(A.N){
        y = A*y;
        if(&J) J = A*J;
    }
    if(a.N) y += a;
  }
  if(norm){
    double l = sqrt(sumOfSqr(y));
    if(&J) J = ~(y/l)*J;
    y = ARR(l);
  }
}

uint TaskMap_LinTrans::dim_phi(const mlr::KinematicWorld& G){
  if(!norm) return a.N;
  if(norm) return 1;
  HALT("");
  return 0;
}
