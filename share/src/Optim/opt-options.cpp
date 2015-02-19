#include "opt-options.h"

//===========================================================================
//
// optimization methods
//

OptOptions::OptOptions() {
  verbose    =MT::getParameter<uint>  ("opt/verbose",1);
  fmin_return=NULL;
  stopTolerance=MT::getParameter<double>("opt/stopTolerance",1e-2);
  stopEvals =MT::getParameter<uint>  ("opt/stopEvals",1000);
  stopIters =MT::getParameter<uint>  ("opt/stopIters",1000);
  initStep  =MT::getParameter<double>("opt/initStep",1.);
  minStep   =MT::getParameter<double>("opt/minStep",-1.);
  maxStep   =MT::getParameter<double>("opt/maxStep",-1.);
  damping   =MT::getParameter<double>("opt/damping",1.);
  stepInc   =MT::getParameter<double>("opt/stepInc",2.);
  stepDec   =MT::getParameter<double>("opt/stepDec",.1);
  dampingInc=MT::getParameter<double>("opt/dampingInc",1.);
  dampingDec=MT::getParameter<double>("opt/dampingDec",1.);
  nonStrictSteps=MT::getParameter<uint>  ("opt/nonStrictSteps",0);
  allowOverstep=MT::getParameter<bool>  ("opt/allowOverstep",false);
  constrainedMethod = (ConstrainedMethodType)MT::getParameter<int>("opt/constrainedMethod",augmentedLag);
  aulaMuInc =MT::getParameter<double>("opt/aulaMuInc",1.);
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
