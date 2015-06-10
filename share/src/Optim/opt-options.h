#pragma once

#include <Core/util.h>

extern uint eval_cost;

enum ConstrainedMethodType { noMethod=0, squaredPenalty, augmentedLag, logBarrier, anyTimeAula };

struct OptOptions {
  uint verbose;
  double *fmin_return;
  double stopTolerance;
  uint   stopEvals;
  uint   stopIters;
  double initStep;
  double minStep;
  double maxStep;
  double damping;
  double stepInc, stepDec;
  double dampingInc, dampingDec;
  int nonStrictSteps; //# of non-strict iterations
  bool allowOverstep;
  ConstrainedMethodType constrainedMethod;
  double aulaMuInc;
  OptOptions();
  void write(std::ostream& os) const;
};
stdOutPipe(OptOptions);

extern Singleton<OptOptions> globalOptOptions;

#define NOOPT (globalOptOptions())