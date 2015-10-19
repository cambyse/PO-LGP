#pragma once

#include <Optim/optimization.h>
#include <Ors/ors.h>

//===========================================================================

struct EffectivePoseProblem:ConstrainedProblemMix{
  ors::KinematicWorld& effKinematics;
  const Graph& KB;
  const Graph& symbolicState_before;
  const Graph& symbolicState_after;
  arr x0;
  int verbose;
  EffectivePoseProblem(ors::KinematicWorld& effKinematics_beforeitial,
                       const Graph& KB, const Graph& symbolicState_before, const Graph& symbolicState_after,
                       int verbose);
  void phi(arr& phi, arr& phiJ, TermTypeA& tt, const arr& x);

  double optimize(arr& x);
};

