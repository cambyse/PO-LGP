#include "taskMap_linTrans.h"

void TaskMap_LinTrans::phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t){
  map->phi(y, J, G, t);
  y = A*y + a;
  if(&J) J = A*J;
}
