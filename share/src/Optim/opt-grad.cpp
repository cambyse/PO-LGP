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

#include <iomanip>

#include "opt-grad.h"

int optGrad(arr& x, const ScalarFunction& f,  OptOptions o) {
  return OptGrad(x, f, o).run();
}

//===========================================================================

OptGrad::OptGrad(arr& _x, const ScalarFunction& _f,  OptOptions _o):
  x(_x), f(_f), o(_o), it(0), evals(0), numTinySteps(0){
  alpha = o.initStep;
  reinit();
}

void OptGrad::reinit(){
  fx = f(gx, NoArr, x);  evals++;

  //startup verbose
  if(o.verbose>1) cout <<"*** optGrad: starting point f(x)=" <<fx <<" alpha=" <<alpha <<endl;
  if(o.verbose>2) cout <<"             x=" <<x <<endl;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;
}

//===========================================================================

OptGrad::StopCriterion OptGrad::step(){
  double fy;
  arr y, gy, Delta;

  it++;
  if(o.verbose>1) cout <<"optGrad it=" <<std::setw(4) <<it <<flush;

  if(!(fx==fx)) HALT("you're calling a gradient step with initial function value = NAN");

  //compute Delta
  Delta = gx / (-length(gx));

  //line search
  uint lineSteps=0;
  for(;;lineSteps++) {
    y = x + alpha*Delta;
    fy = f(gy, NoArr, y);  evals++;
    if(o.verbose>2) cout <<" \tprobing y=" <<y;
    if(o.verbose>1) cout <<" \tevals=" <<std::setw(4) <<evals <<" \talpha=" <<std::setw(11) <<alpha <<" \tf(y)=" <<fy <<flush;
    bool wolfe = (fy <= fx + o.wolfe*alpha*scalarProduct(Delta,gx) );
    if(fy==fy && (wolfe || o.nonStrictSteps==-1 || o.nonStrictSteps>(int)it)) { //fy==fy is for NAN?
      //accept new point
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      if(fx-fy<o.stopFTolerance || alpha<o.stopTolerance) numTinySteps++; else numTinySteps=0;
      x = y;
      fx = fy;
      gx = gy;
      if(wolfe){
        alpha *= o.stepInc;
      }else{
        alpha *= o.stepDec;
      }
      break;
    } else {
      //reject new point
      if(o.verbose>1) cout <<" - reject" <<flush;
      if(lineSteps>o.stopLineSteps) break;
      if(evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
      if(o.verbose>1) cout <<"\n  (line search)" <<flush;
      alpha *= o.stepDec;
    }
  }

  if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;

  //stopping criteria
#define STOPIF(expr, code, ret) if(expr){ if(o.verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; code; return stopCriterion=ret; }
  //  STOPIF(absMax(Delta)<o.stopTolerance, , stopCrit1);
  STOPIF(numTinySteps>o.stopTinySteps, numTinySteps=0, stopCrit2);
  //  STOPIF(alpha<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(lineSteps>=o.stopLineSteps, , stopCritLineSteps);
  STOPIF(evals>=o.stopEvals, , stopCritEvals);
  STOPIF(it>=o.stopIters, , stopCritEvals);
#undef STOPIF

  return stopCriterion=stopNone;
}


OptGrad::~OptGrad(){
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
#ifndef MLR_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, true);
#endif
  if(o.verbose>1) cout <<"--- OptGradStop: f(x)=" <<fx <<endl;
}


OptGrad::StopCriterion OptGrad::run(uint maxIt){
  numTinySteps=0;
  for(uint i=0;i<maxIt;i++){
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion==stopCritLineSteps){ reinit();   continue; }
    if(stopCriterion>=stopCrit1) break;
  }
  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, false);
  if(o.fmin_return) *o.fmin_return= fx;
  return stopCriterion;
}
