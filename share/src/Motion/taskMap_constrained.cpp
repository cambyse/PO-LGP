#include "taskMap_constrained.h"

void CollisionConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  G.kinematicsProxyCost(y, J, 2.*margin, false);
  y -= .9;
}

void PlaneConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  int body_i = G.shapes(i)->body->index;
  ors::Vector vec_i = G.shapes(i)->rel.pos;

  arr y_eff, J_eff;
  G.kinematicsPos(y_eff, (&J?J_eff:NoArr), body_i, &vec_i);

  y_eff.append(1.); //homogeneous coordinates
  if(&J) J_eff.append(zeros(1,J_eff.d1));

  y.resize(1);
  y(0) = scalarProduct(y_eff, planeParams);
  if(&J) J = ~planeParams * J_eff;
}
