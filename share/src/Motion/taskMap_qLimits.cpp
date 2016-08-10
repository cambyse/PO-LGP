#include "taskMap_qLimits.h"

//===========================================================================

void TaskMap_qLimits::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  if(!limits.N) limits=G.getLimits();
  G.kinematicsLimitsCost(y, J, limits);
}

