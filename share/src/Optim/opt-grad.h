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

#include <Core/array.h>
#include "opt-options.h"

int optGrad(arr& x, const ScalarFunction& f, OptOptions opt=NOOPT);

struct OptGrad{
  arr& x;
  ScalarFunction f;
  OptOptions o;

  enum StopCriterion { stopNone=0, stopCrit1, stopCrit2, stopCritLineSteps, stopCritEvals, stopStepFailed };
  double fx;
  arr gx;
  double alpha;
  uint it, evals, numTinySteps;
  StopCriterion stopCriterion;
  ofstream fil;

  OptGrad(arr& x, const ScalarFunction& f, OptOptions o=NOOPT);
  ~OptGrad();
  StopCriterion step();
  StopCriterion run(uint maxIt = 1000);
  void reinit();
};
