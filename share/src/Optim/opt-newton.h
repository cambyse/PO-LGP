#pragma once

#include "optimization.h"

int optNewton(arr& x, ScalarFunction& f, OptOptions opt);

struct OptNewton{
  arr& x;
  ScalarFunction& f;
  OptOptions o;
  arr *additionalRegularizer;

  enum StopCriterion { stopNone=0, stopCrit1, stopCrit2, stopCritEvals };
  double fx;
  arr gx, Hx;
  double alpha, lambda;
  uint it, evals;
  bool x_changed;
  ofstream fil;

  OptNewton(arr& x, ScalarFunction& f, OptOptions o);
  ~OptNewton();
  StopCriterion step();
  StopCriterion run();
  void reinit();
};
