#pragma once

#include <Optim/optimization.h>
#include <Ors/ors.h>

//===========================================================================

struct EffectivePoseProblem:ConstrainedProblemMix{
  ors::KinematicWorld& effKinematics;
  const Graph& symbolicState;
  int verbose;
  EffectivePoseProblem(ors::KinematicWorld& effKinematics_initial,
                       const Graph& symbolics,
                       int verbose);
  void phi(arr& phi, arr& phiJ, TermTypeA& tt, const arr& x);

  double optimize(arr& x);
};

