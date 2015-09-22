/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include <iomanip>

#include "opt-newton.h"

bool sanityCheck=false; //true;

/** @brief Minimizes \f$f(x) = A(x)^T x A^T(x) - 2 a(x)^T x + c(x)\f$. The optional _user arguments specify,
 * if f has already been evaluated at x (another initial evaluation is then omitted
 * to increase performance) and the evaluation of the returned x is also returned */
int optNewton(arr& x, const ScalarFunction& f,  OptOptions o) {
  return OptNewton(x, f, o).run();
}

//===========================================================================
//
// OptNewton
//

OptNewton::OptNewton(arr& _x, const ScalarFunction& _f,  OptOptions _o):
  x(_x), f(_f), o(_o){
  alpha = o.initStep;
  beta = o.damping;
  it=0;
  evals=0;
  additionalRegularizer=NULL;
  reinit();
}

void OptNewton::reinit(){
  fx = f(gx, Hx, x);  evals++;
  if(additionalRegularizer)  fx += scalarProduct(x,(*additionalRegularizer)*vectorShaped(x));

  //startup verbose
  if(o.verbose>1) cout <<"*** optNewton: starting point f(x)=" <<fx <<" alpha=" <<alpha <<" beta=" <<beta <<endl;
  if(o.verbose>2) cout <<"\nx=" <<x <<endl;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;
}

OptNewton::StopCriterion OptNewton::step(){
  double fy;
  arr y, gy, Hy, Delta;
//  x_changed=false;

  it++;
  if(o.verbose>1) cout <<"optNewton it=" <<std::setw(3) <<it << " \tbeta=" <</*std::setprecision(3) <<*/beta <<flush;

  if(!(fx==fx)) HALT("you're calling a newton step with initial function value = NAN");

  //compute Delta
  arr R=Hx;
  if(beta) { //Levenberg Marquardt damping
    if(R.special==arr::RowShiftedPackedMatrixST) for(uint i=0; i<R.d0; i++) R(i,0) += beta; //(R(i,0) is the diagonal in the packed matrix!!)
    else for(uint i=0; i<R.d0; i++) R(i,i) += beta;
  }
  if(additionalRegularizer) {
    if(R.special==arr::RowShiftedPackedMatrixST) R = unpack(R);
    //      cout <<*addRegularizer <<R <<endl;
    Delta = lapack_Ainv_b_sym(R + (*additionalRegularizer), -(gx+(*additionalRegularizer)*vectorShaped(x)));
  } else {
    try {
      Delta = lapack_Ainv_b_sym(R, -gx);
    }catch(...){
      if(o.verbose>0) cout <<endl <<"** hessian inversion failed ... increasing damping **" <<endl;
      beta *= 10.;
      return stopCriterion=stopStepFailed;
    }
  }
  if(o.maxStep>0. && absMax(Delta)>o.maxStep)  Delta *= o.maxStep/absMax(Delta);
  if(o.verbose>1) cout <<" \t|Delta|=" <<absMax(Delta) <<flush;

  //lazy stopping criterion: stop without any update
  if(absMax(Delta)<1e-1*o.stopTolerance){
    if(o.verbose>1) cout <<" \t - NO UPDATE" <<endl;
    return stopCriterion=stopCrit1;
  }

  for(;;) { //stepsize adaptation loop -- doesn't iterate for dampingInc (LV) option
    y = x + alpha*Delta;
    fy = f(gy, Hy, y);  evals++;
    if(additionalRegularizer) fy += scalarProduct(y,(*additionalRegularizer)*vectorShaped(y));
    if(o.verbose>2) cout <<" \tprobing y=" <<y;
    if(o.verbose>1) cout <<" \tevals=" <<evals <<" \talpha=" <<alpha <<" \tf(y)=" <<fy  /*<<" \tf(y)-f(x)=" <<fy-fx */<<flush;
    //CHECK_EQ(fy,fy, "cost seems to be NAN: ly=" <<fy);
    if(fy==fy && (fy <= fx || o.nonStrictSteps==-1 || o.nonStrictSteps>(int)it)) { //fy==fy is for NAN?
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      //adopt new point and adapt stepsize|damping
      x = y;
      fx = fy;
      gx = gy;
      Hx = Hy;
      if(fy<=fx){
        if(alpha>.9) beta *= o.dampingDec;
        alpha *= o.stepInc;
        if(!o.allowOverstep) if(alpha>1.) alpha=1.;
      }else{
        beta *= o.dampingInc;
        alpha *= o.stepDec;
      }
      break;
    } else {
      if(o.verbose>1) cout <<" - reject" <<std::endl <<"\t\t\t\t";
      //reject new points and adapte stepsize|damping
      if(alpha*absMax(Delta)<1e-3*o.stopTolerance || evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
      beta = beta*o.dampingInc;
      alpha = alpha*o.stepDec;
      if(o.dampingInc!=1.) break; //we need to recompute Delta
    }
  }

  if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<alpha;
  if(o.verbose>2) fil <<' ' <<x;
  if(o.verbose>0) fil <<endl;

  //stopping criteria
#define STOPIF(expr, ret) if(expr){ if(o.verbose>1) cout <<"\t\t\t\t\t\t--- stopping criterion='" <<#expr <<"'" <<endl; return stopCriterion=ret; }
  STOPIF(absMax(Delta)<o.stopTolerance, stopCrit1);
  STOPIF(alpha*absMax(Delta)<1e-3*o.stopTolerance, stopCrit2);
  STOPIF(evals>=o.stopEvals, stopCritEvals);
  STOPIF(it>=o.stopIters, stopCritEvals);
#undef STOPIF

  return stopCriterion=stopNone;
}


OptNewton::~OptNewton(){
  if(o.fmin_return) *o.fmin_return=fx;
  if(o.verbose>0) fil.close();
#ifndef MT_MSVC
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, true);
#endif
  if(o.verbose>1) cout <<"--- optNewtonStop: f(x)=" <<fx <<endl;
}


OptNewton::StopCriterion OptNewton::run(){
  for(;;){
    step();
    if(stopCriterion==stopStepFailed) continue;
    if(stopCriterion>=stopCrit1) break;
  }
  return stopCriterion;
}
