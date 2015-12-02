#include "opt-options.h"

//===========================================================================
//
// optimization methods
//

OptOptions::OptOptions() {
  verbose    = mlr::getParameter<uint>  ("opt/verbose", 1);
  fmin_return=NULL;
  stopTolerance= mlr::getParameter<double>("opt/stopTolerance", 1e-2);
  stopEvals = mlr::getParameter<uint>  ("opt/stopEvals", 1000);
  stopIters = mlr::getParameter<uint>  ("opt/stopIters", 1000);
  initStep  = mlr::getParameter<double>("opt/initStep", 1.);
  minStep   = mlr::getParameter<double>("opt/minStep", -1.);
  maxStep   = mlr::getParameter<double>("opt/maxStep", -1.);
  damping   = mlr::getParameter<double>("opt/damping", 1.);
  stepInc   = mlr::getParameter<double>("opt/stepInc", 2.);
  stepDec   = mlr::getParameter<double>("opt/stepDec", .1);
  dampingInc= mlr::getParameter<double>("opt/dampingInc", 1.);
  dampingDec= mlr::getParameter<double>("opt/dampingDec", 1.);
  nonStrictSteps= mlr::getParameter<uint>  ("opt/nonStrictSteps", 0);
  allowOverstep= mlr::getParameter<bool>  ("opt/allowOverstep", false);
  constrainedMethod = (ConstrainedMethodType)mlr::getParameter<int>("opt/constrainedMethod", augmentedLag);
  aulaMuInc = mlr::getParameter<double>("opt/aulaMuInc", 1.);
}

void OptOptions::write(std::ostream& os) const{
#define WRT(x) os <<#x <<" = " <<x <<endl;
  WRT(verbose);
//  double *fmin_return);
  WRT(stopTolerance);
  WRT(stopEvals);
  WRT(stopIters);
  WRT(initStep);
  WRT(minStep);
  WRT(maxStep);
  WRT(damping);
  WRT(stepInc);
  WRT(stepDec);
  WRT(dampingInc);
  WRT(dampingDec);
  WRT(nonStrictSteps);
  WRT(allowOverstep);
  WRT(constrainedMethod);
  WRT(aulaMuInc);
#undef WRT
}
