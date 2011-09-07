#include "optimization.h"

#ifndef CHECK_EPS
#  define CHECK_EPS 1e-8
#endif

uint GaussNewton(arr& x, double tolerance, GaussNewtonCostFunction& f, uint maxEvals){
  double a=1.;
  double lx, ly;
  arr Delta, y;
  arr R(x.N, x.N), r(x.N);
  uint evals=0;
  
  //compute initial costs
  f.calcTermsAt(x);  evals++;
  lx = sumOfSqr(f.phi);
  VERBOSE(2, cout  <<"starting point x="  <<x  <<" l(x)="  <<lx  <<" a=" <<a  <<endl);
  
  for(;;){
    //compute Delta
    arr tmp;
    innerProduct(R, ~f.J, f.J);  R.reshape(x.N, x.N);
    innerProduct(r, ~f.J, f.phi);
    
    lapack_Ainv_b_sym(Delta, R, -r);
    
    for(;;){
      y = x + a*Delta;
      f.calcTermsAt(y);  evals++;
      ly = sumOfSqr(f.phi);
      VERBOSE(2, cout  <<evals  <<" \tprobing y="  <<y  <<" \tl(y)="  <<ly  <<" \t|Delta|="  <<norm(Delta)  <<" \ta=" <<a);
      CHECK(ly==ly, "cost seems to be NAN: ly="  <<ly);
      if(ly <= lx) break;
      if(evals>maxEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
      //decrease stepsize
      a = .5*a;
      VERBOSE(2, cout  <<" - reject"  <<endl);
    }
    VERBOSE(2, cout  <<" - ACCEPT"  <<endl);
    
    //adopt new point and adapt stepsize
    x = y;
    lx = ly;
    a = pow(a, 0.7);
    
    //stopping criterion
    if(norm(Delta)<tolerance || evals>maxEvals) break;
  }
  //cout  <<lx  <<' '  <<flush;
  return evals;
}

void checkGradient(OptimizationProblem &p,
                   const arr& x, double tolerance){
  arr J, dx, JJ;
  double y, dy;
  y=p.f(&J, x);
  
  JJ.resize(x.N);
  double eps=CHECK_EPS;
  uint i;
  for(i=0; i<x.N; i++){
    dx=x;
    dx.elem(i) += eps;
    dy = p.f(NULL, dx);
    dy = (dy-y)/eps;
    JJ(i)=dy;
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, 0);
//   MT::save(J, "z.J");
//   MT::save(JJ, "z.JJ");
  if(md>tolerance){
    MT_MSG("checkGradient -- FAILURE -- max diff="  <<md  <<" (stored in files z.J and z.JJ)");
    MT::save(J, "z.J");
    MT::save(JJ, "z.JJ");
    //cout  <<"\nmeasured grad="  <<JJ  <<"\ncomputed grad="  <<J  <<endl;
    HALT("");
  }else{
    cout  <<"checkGradient -- SUCCESS (max diff error="  <<md  <<")"  <<endl;
  }
}

void checkGradient_vec(OptimizationProblem &p,
                       const arr& x, double tolerance){
  arr y, J, dx, dy, JJ;
  p.F(y, &J, x);
  
  JJ.resize(y.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++){
    dx=x;
    dx.elem(i) += eps;
    p.F(dy, NULL, dx);
    dy = (dy-y)/eps;
    for(k=0; k<y.N; k++) JJ(k, i)=dy.elem(k);
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
//   MT::save(J, "z.J");
//   MT::save(JJ, "z.JJ");
  if(md>tolerance){
    MT_MSG("checkGradient -- FAILURE -- max diff="  <<md  <<" (stored in files z.J and z.JJ)");
    MT::save(J, "z.J");
    MT::save(JJ, "z.JJ");
    //cout  <<"\nmeasured grad="  <<JJ  <<"\ncomputed grad="  <<J  <<endl;
    //HALT("");
  }else{
    cout  <<"checkGradient -- SUCCESS (max diff error="  <<md  <<")"  <<endl;
  }
}


//===========================================================================
//
// Rprop
//

int _sgn(double x){ if(x > 0) return 1; if(x < 0) return -1; return 0; }
double _mymin(double x, double y){ return x < y ? x : y; }
double _mymax(double x, double y){ return x > y ? x : y; }


Rprop::Rprop(){
  incr   = 1.2;
  decr   = .33;
  dMax = 50;
  dMin = 1e-6;
  rMax = 0;
  delta0 = 1.;
}

void Rprop::init(double _delta0){
  stepSize.resize(0);
  lastGrad.resize(0);
  delta0 = _delta0;
}

bool Rprop::done(){
  double maxStep = stepSize(stepSize.maxIndex());
  return maxStep < incr*dMin;
}

void Rprop::step(double& w, const double& grad){
  static arr W, GRAD;
  W.referTo(&w, 1); GRAD.referTo(&grad, 1);
  step(W, GRAD);
}

void Rprop::step(arr& w, const arr& grad, uint *singleI){
  if(!stepSize.N){ //initialize
    stepSize.resize(w.N);
    lastGrad.resize(w.N);
    lastGrad.setZero();
    stepSize = delta0;
  }
  CHECK(grad.N==stepSize.N, "Rprop: gradient dimensionality changed!");
  CHECK(w.N==stepSize.N   , "Rprop: parameter dimensionality changed!");
  
  uint i=0, I=w.N;
  if(singleI){ i=*(singleI); I=i+1; }
  for(; i<I; i++){
    if(grad.elem(i) * lastGrad(i) > 0){  //same direction as last time
      if(rMax) dMax=fabs(rMax*w.elem(i));
      stepSize(i) = _mymin(dMax, incr * stepSize(i)); //increase step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    } else if(grad.elem(i) * lastGrad(i) < 0){  //change of direction
      stepSize(i) = _mymax(dMin, decr * stepSize(i)); //decrease step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = 0;                               //memorize to continue below next time
    }else{                                  //after change of direcion
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    }
  }
}

void Rprop::step(arr& x, OptimizationProblem& p){
  arr grad;
  p.f(&grad, x);
  step(x, grad);
}

//----- the rprop wrapped with stopping criteria
int Rprop::loop(arr& _x,
                OptimizationProblem& p,
                double *fmin_return,
                double stoppingTolerance,
                uint maxIterations){
  arr x, J(_x.N), xmin;
  double y, ymin=0;
  uint lost_steps=0, small_steps=0;
  x=_x;
  
  uint i;
  for(i=0; i<maxIterations; i++){
    //checkGradient(p, x, stoppingTolerance);
    //compute value and gradient at x
    y = p.f(&J, x);
    //update best-so-far
    if(!i){ ymin=y; xmin=x; }
    if(y<ymin){
      ymin=y; xmin=x;
      lost_steps=0;
    }else{
      lost_steps++;
      if(lost_steps>10){
        stepSize*=(double).1;
        lastGrad=(double)0.;
        x=xmin;
        lost_steps=0;
      }
    }
    //update x
    step(x, J);
    //check stopping criterion based on step-length in x
    double diff=maxDiff(x, xmin);
    //cout  <<"RPROP iter= " <<i  <<"  x-diff= "  <<diff  <<"  f= "  <<y  <<endl;
    if(diff<stoppingTolerance){ small_steps++; }else{ small_steps=0; }
    if(small_steps>10)  break;
  }
  if(fmin_return) *fmin_return=ymin;
  _x=xmin;
  return i;
}


#ifdef MT_GSL
#include <gsl/gsl_cdf.h>
bool DecideSign::step(double x){
  N++;
  sumX+=x;
  sumXX+=x*x;
  if(N<=10) return false;
  if(!sumX) return true;
  double m=sumX/N;
  double s=sqrt((sumXX-sumX*m)/(N-1));
  double t=sqrt(N)*fabs(m)/s;
  double T=gsl_cdf_tdist_Pinv(1.-1e-6, N-1); //decide with error-prob 1e-4 if sign is significant
  if(t>T) return true;
  return false;
}
#else
bool DecideSign::step(double x){ NIY; }
#endif
