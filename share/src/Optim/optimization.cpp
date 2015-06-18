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


#include "optimization.h"

uint eval_cost=0;
//SqrPotential& NoPot = *((SqrPotential*)NULL);
//PairSqrPotential& NoPairPot = *((PairSqrPotential*)NULL);
Singleton<OptOptions> globalOptOptions;
TermTypeA& NoTermTypeA = *((TermTypeA*)NULL);


//===========================================================================
//
// misc (internal)
//

//void init(SqrPotential &V, uint n) { V.A.resize(n,n); V.a.resize(n); V.A.setZero(); V.a.setZero(); V.c=0.; }


//documentations... TODO: move! but not in header!

/// A scalar function $y = f(x)$, if @grad@ is not NoArr, gradient is returned
typedef std::function<double(arr& g, arr& H, const arr& x)> ScalarFunction;

/// A vector function $y = f(x)$, if @J@ is not NoArr, Jacobian is returned
/// This also implies an optimization problem $\hat f(y) = y^T(x) y(x)$ of (iterated)
/// Gauss-Newton type where the Hessian is approximated by J^T J
typedef std::function<void(arr& y, arr& J, const arr& x)> VectorFunction;

/// A scalar function $y = f(x)$, if @S@ is non-NULL, local quadratic approximation is returned
/// This also implies an optimization problem of (iterated) Newton
/// type with the given Hessian
struct QuadraticFunction;


//===========================================================================
//
// checks, evaluation and converters
//

double evaluateSF(ScalarFunction& f, const arr& x) {
  return f(NoArr, NoArr, x);
}

double evaluateVF(VectorFunction& f, const arr& x) {
  arr y;
  f(y, NoArr, x);
  return sumOfSqr(y);
}


bool checkAllGradients(const ConstrainedProblem &P, const arr& x, double tolerance){
  ScalarFunction F = [&P](arr& df, arr& Hf, const arr& x){
    return P(df, Hf, NoArr, NoArr, NoArr, NoArr, x);
  };
  VectorFunction G = [&P](arr& g, arr& Jg, const arr& x){
    return P(NoArr, NoArr, g, Jg, NoArr, NoArr, x);
  };
  VectorFunction H = [&P](arr& h, arr& Jh, const arr& x){
    return P(NoArr, NoArr, NoArr, NoArr, h, Jh, x);
  };

  bool good=true;
  cout <<"f-gradient: "; good &= checkGradient(F, x, tolerance);
  cout <<"f-hessian:  "; good &= checkHessian (F, x, tolerance);
  cout <<"g-jacobian: "; good &= checkJacobian(G, x, tolerance);
  cout <<"h-jacobian: "; good &= checkJacobian(H, x, tolerance);
  return good;
}

bool checkJacobianCP(const ConstrainedProblemMix &P, const arr& x, double tolerance){
  VectorFunction F = [&P](arr& phi, arr& J, const arr& x){
    return P(phi, J, NoTermTypeA, x);
  };
  return checkJacobian(F, x, tolerance);
}

//===========================================================================
//
// helpers
//

void displayFunction(const ScalarFunction &f, bool wait, double lo, double hi){
  arr X, Y;
  X.setGrid(2,lo,hi,100);
  Y.resize(X.d0);
  for(uint i=0;i<X.d0;i++){
    double fx=f(NoArr, NoArr, X[i]);
    Y(i) = ((fx==fx && fx<10.)? fx : 10.);
  }
  Y.reshape(101,101);
//  plotGnuplot();  plotSurface(Y);  plot(true);
  write(LIST<arr>(Y),"z.fct");
  gnuplot("reset; splot [-1:1][-1:1] 'z.fct' matrix us ($1/50-1):($2/50-1):3 w l", wait, true);
}



/// minimizes \f$f(x)\f$ using its gradient only
uint optGradDescent(arr& x, const ScalarFunction& f, OptOptions o) {
  uint evals=0;
  arr y, grad_x, grad_y;
  double fx, fy;
  double a=o.initStep;
  
  fx = f(grad_x, NoArr, x);  evals++;
  if(o.verbose>1) cout <<"*** optGradDescent: starting point x=" <<(x.N<20?x:ARR()) <<" f(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
  
  grad_x /= length(grad_x);
  
  for(uint k=0;; k++) {
    y = x - a*grad_x;
    fy = f(grad_y, NoArr, y);  evals++;
    CHECK_EQ(fy,fy, "cost seems to be NAN: fy=" <<fy);
    if(o.verbose>1) cout <<"optGradDescent " <<evals <<' ' <<eval_cost <<" \tprobing y=" <<(y.N<20?y:ARR()) <<" \tf(y)=" <<fy <<" \t|grad|=" <<length(grad_y) <<" \ta=" <<a;
    
    if(fy <= fx) {
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      double step=length(x-y);
      x = y;
      fx = fy;
      grad_x = grad_y/length(grad_y);
      a *= 1.2;
      if(o.maxStep>0. && a>o.maxStep) a = o.maxStep;
      if(o.verbose>0) fil <<evals <<' ' <<eval_cost <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
      if(step<o.stopTolerance) break;
    } else {
      if(o.verbose>1) cout <<" - reject" <<endl;
      a *= .5;
    }
    if(evals>o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
    if(k>o.stopIters) break;
  }
  if(o.verbose>0) fil.close();
  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l",NULL,true);
  return evals;
}


RUN_ON_INIT_BEGIN(optimization)
TermTypeA::memMove=true;
RUN_ON_INIT_END(optimization)
