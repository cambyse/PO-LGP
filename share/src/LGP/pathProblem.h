#pragma once

#include <Optim/optimization.h>
#include <Motion/motion.h>

//===========================================================================

struct PathProblem:ConstrainedProblem{
  ors::KinematicWorld world;
  const Graph& symbolicState;
  uint microSteps;
  int verbose;

  MotionProblem MP;
  MotionProblemFunction MPF;

  PathProblem(const ors::KinematicWorld& world_initial,
              const ors::KinematicWorld& world_final,
              const Graph& symbolicState,
              uint microSteps,
              int verbose);

  double optimize(arr& x);
};
