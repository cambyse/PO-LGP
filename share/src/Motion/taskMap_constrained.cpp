#include "taskMap_constrained.h"

void CollisionConstraint::phi(arr& y, arr& J, const ors::Graph& G){
  G.phiCollision(y, J, 2.*margin);
  y -= .9;
}
