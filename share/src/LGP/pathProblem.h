/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


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

  PathProblem(const ors::KinematicWorld& world_initial,
              const ors::KinematicWorld& world_final,
              const Graph& symbolicState,
              uint microSteps,
              int verbose);

  double optimize(arr& x);
};
