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

#include <Core/util.h>

extern uint eval_cost;

enum ConstrainedMethodType { noMethod=0, squaredPenalty, augmentedLag, logBarrier, anyTimeAula, squaredPenaltyFixed };

struct OptOptions {
  uint verbose;
  double *fmin_return;
  double stopTolerance;
  double stopFTolerance;
  double stopGTolerance;
  uint   stopEvals;
  uint   stopIters;
  uint   stopLineSteps;
  uint   stopTinySteps;
  double initStep;
  double minStep;
  double maxStep;
  double damping;
  double stepInc, stepDec;
  double dampingInc, dampingDec;
  double wolfe;
  int nonStrictSteps; //# of non-strict iterations
  bool allowOverstep;
  ConstrainedMethodType constrainedMethod;
  double muInit, muLBInit;
  double aulaMuInc;
  OptOptions();
  void write(std::ostream& os) const;
};
stdOutPipe(OptOptions);

extern Singleton<OptOptions> globalOptOptions;

#define NOOPT (globalOptOptions())
