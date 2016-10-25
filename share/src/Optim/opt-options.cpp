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


#include "opt-options.h"

//===========================================================================
//
// optimization methods
//

OptOptions::OptOptions() {
  verbose    = mlr::getParameter<uint>  ("opt/verbose", 1);
  fmin_return=NULL;
  stopTolerance= mlr::getParameter<double>("opt/stopTolerance", 1e-2);
  stopFTolerance= mlr::getParameter<double>("opt/stopFTolerance", 1e-1);
  stopGTolerance= mlr::getParameter<double>("opt/stopGTolerance", -1.);
  stopEvals = mlr::getParameter<uint>  ("opt/stopEvals", 1000);
  stopIters = mlr::getParameter<uint>  ("opt/stopIters", 1000);
  stopLineSteps = mlr::getParameter<uint>  ("opt/stopLineSteps", 10);
  stopTinySteps = mlr::getParameter<uint>  ("opt/stopTinySteps", 10);
  initStep  = mlr::getParameter<double>("opt/initStep", 1.);
  minStep   = mlr::getParameter<double>("opt/minStep", -1.);
  maxStep   = mlr::getParameter<double>("opt/maxStep", .5);
  damping   = mlr::getParameter<double>("opt/damping", .1);
  stepInc   = mlr::getParameter<double>("opt/stepInc", 2.);
  stepDec   = mlr::getParameter<double>("opt/stepDec", .1);
  dampingInc= mlr::getParameter<double>("opt/dampingInc", 2.);
  dampingDec= mlr::getParameter<double>("opt/dampingDec", .5);
  wolfe     = mlr::getParameter<double>("opt/wolfe", .01);
  nonStrictSteps= mlr::getParameter<uint>  ("opt/nonStrictSteps", 0);
  allowOverstep= mlr::getParameter<bool>  ("opt/allowOverstep", true);
  constrainedMethod = (ConstrainedMethodType)mlr::getParameter<int>("opt/constrainedMethod", anyTimeAula);
  muInit = mlr::getParameter<double>("opt/muInit", 1.);
  muLBInit = mlr::getParameter<double>("opt/muLBInit", 1.);
  aulaMuInc = mlr::getParameter<double>("opt/aulaMuInc", 2.);
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
