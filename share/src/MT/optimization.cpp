#include "optimization.h"

#ifndef CHECK_EPS
#  define CHECK_EPS 1e-8
#endif


double SqrChainFunction::f_total(const arr& x){
  double cost=0.;
  uint T=x.d0;
  cost += fi(NULL, 0, x[0]);
  for(uint t=1;t<T;t++){
    cost += fi (NULL, t, x[t]);
    cost += fij(NULL, t-1, t, x[t-1], x[t]);
  }
  return cost;
}


void init(SqrPotential &V, uint n){ V.A.resize(n,n); V.a.resize(n); V.A.setZero(); V.a.setZero(); V.hata=0.; }

uint optDynamicProgramming(arr& x, SqrChainFunction& f, double *fmin_return, double stoppingTolerance, uint maxEvals, double maxStepSize, uint verbose){

  uint T=x.d0,n=x.d1;
  uint evals=0;
  arr y;
  y.resizeAs(x);

  MT::Array<SqrPotential> V(T);
  arr Bbarinv(T,n,n);
  arr bbar(T,n);
  arr tmp;

  double fx = f.f_total(x);
  cout <<fx <<endl;
  
  for(uint k=0;k<10;k++){
    //backward
    init(V(T-1),n);
    SqrPotential fi;
    PairSqrPotential fij;
    //backward
    for(uint t=T-1;t>0;t--){
      f.fi (&fi , t, x[t]);
      f.fij(&fij, t-1, t, x[t-1], x[t]);
      arr Bbar = fij.B + fi.A + V(t).A;
      bbar[t] = fij.b + fi.a + V(t).a;
      double hatabar = fij.hata + fi.hata + V(t).hata;
      inverse_SymPosDef(Bbarinv[t](), Bbar);
      V(t-1).hata = hatabar - (~bbar[t] * Bbarinv[t] * bbar[t])(0);
      tmp  = fij.C*Bbarinv[t];
      V(t-1).a = fij.a - tmp * bbar[t];
      V(t-1).A = fij.A - tmp * ~fij.C;
    }
    //forward
    inverse_SymPosDef(tmp, V(0).A);
    y[0] = tmp*V(0).a;
    for(uint t=1;t<T;t++){
      y[t] = Bbarinv[t]*(bbar[t] - ~fij.C*y[t-1]);
    }
  
    double fy=V(0).hata;
    double fy_bla = f.f_total(y);
    if(fy<=fx){
      x=y;
      fx=fy;
    }
    
    cout <<fx <<' ' <<fy <<endl;
    //break;
  }
  return evals;
}

/*double ScalarGraphFunction::f_total(const arr& X){
  uint n=X.d0;
  uintA E=edges();
  double f=0.;
  for(uint i=0;i<n;i++)    f += fi(NULL, i, X[i]);
  for(uint i=0;i<E.d0;i++) f += fij(NULL, NULL, E(i,0), E(i,1), X[E(i,0)], X[E(i,1)]);
  return f;
}

struct Tmp:public ScalarFunction{
  ScalarGraphFunction *sgf;

  double fs(arr* grad, const arr& X){
    uint n=X.d0,i,j,k;
    uintA E=sgf->edges();
    double f=0.;
    arr gi, gj;
    if(grad) (*grad).resizeAs(X);
    for(i=0;i<n;i++){
      f += sgf->fi((grad?&gi:NULL), i, X[i]);
      if(grad) (*grad)[i]() += gi;
    }
    for(k=0;k<E.d0;k++){
      i=E(k,0);
      j=E(i,1);
      f += sgf->fij((grad?&gi:NULL), (grad?&gj:NULL), i, j, X[i], X[j]);
      if(grad) (*grad)[i]() += gi;
      if(grad) (*grad)[j]() += gj;
    }
    return f;
  }
};

Tmp convert_ScalarFunction(ScalarGraphFunction& f){
  Tmp tmp;
  tmp.sgf=&f;
  return tmp;
}*/


void checkGradient(ScalarFunction &f,
                   const arr& x, double tolerance){
  arr J, dx, JJ;
  double y, dy;
  y=f.fs(&J, x);
  
  JJ.resize(x.N);
  double eps=CHECK_EPS;
  uint i;
  for(i=0; i<x.N; i++){
    dx=x;
    dx.elem(i) += eps;
    dy = f.fs(NULL, dx);
    dy = (dy-y)/eps;
    JJ(i)=dy;
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, 0);
//   MT::save(J, "z.J");
//   MT::save(JJ, "z.JJ");
  if(md>tolerance){
    MT_MSG("checkGradient (scalar) -- FAILURE -- max diff=" <<md <<" (stored in files z.J and z.JJ)");
    MT::save(J, "z.J");
    MT::save(JJ, "z.JJ");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    HALT("");
  }else{
    cout <<"checkGradient (scalar) -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
}

void checkGradient(VectorFunction &f,
                   const arr& x, double tolerance){
  arr y, J, dx, dy, JJ;
  f.fv(y, &J, x);
  
  JJ.resize(y.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++){
    dx=x;
    dx.elem(i) += eps;
    f.fv(dy, NULL, dx);
    dy = (dy-y)/eps;
    for(k=0; k<y.N; k++) JJ(k, i)=dy.elem(k);
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
//   MT::save(J, "z.J");
//   MT::save(JJ, "z.JJ");
  if(md>tolerance){
    MT_MSG("checkGradient (vector) -- FAILURE -- max diff=" <<md <<" (stored in files z.J and z.JJ)");
    MT::save(J, "z.J");
    MT::save(JJ, "z.JJ");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
  }else{
    cout <<"checkGradient (vector) -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
}

uint optRprop(arr& x, ScalarFunction& f, double initialStepSize, double *fmin_return, double stoppingTolerance, uint maxEvals, uint verbose ){
  Rprop rp;
  rp.init(initialStepSize);
  return rp.loop(x, f, fmin_return, stoppingTolerance, maxEvals, verbose);
}

uint optGaussNewton(arr& x, VectorFunction& f, double *fmin_return, double stoppingTolerance, uint maxEvals, double maxStepSize, uint verbose){
  double a=1.;
  double fx, fy;
  arr Delta, y;
  arr R(x.N, x.N), r(x.N);
  uint evals=0;
  
  //compute initial costs
  arr phi, J;
  f.fv(phi, &J, x);  evals++;
  fx = sumOfSqr(phi);
  if(verbose>1) cout <<"*** optGaussNewton: starting point x=" <<x <<" l(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(verbose>0) fil.open("z.gaussNewton");
  if(verbose>0) fil <<0 <<' ' <<fx <<' ' <<a <<endl;
  
  for(;;){
    //compute Delta
    arr tmp;
    innerProduct(R, ~J, J);  R.reshape(x.N, x.N);
    innerProduct(r, ~J, phi);
    
    lapack_Ainv_b_sym(Delta, R, -r);
    if(maxStepSize>0. && norm(Delta)>maxStepSize)
      Delta *= maxStepSize/norm(Delta);
    
    for(;;){
      y = x + a*Delta;
      f.fv(phi, &J, y);  evals++;
      fy = sumOfSqr(phi);
      if(verbose>1) cout <<evals <<" \tprobing y=" <<y <<" \tl(y)=" <<fy <<" \t|Delta|=" <<norm(Delta) <<" \ta=" <<a;
      CHECK(fy==fy, "cost seems to be NAN: ly=" <<fy);
      if(fy <= fx) break;
      if(evals>maxEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
      //decrease stepsize
      a = .1*a;
      if(verbose>1) cout <<" - reject" <<endl;
    }
    if(verbose>1) cout <<" - ACCEPT" <<endl;
    
    //adopt new point and adapt stepsize
    x = y;
    fx = fy;
    a = pow(a, 0.5);
    if(verbose>0) fil <<evals <<' ' <<fx <<' ' <<a <<endl;
    
    //stopping criterion
    if(norm(Delta)<stoppingTolerance || evals>maxEvals) break;
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.gaussNewton' us 1:2 w l");
  return evals;
}

uint optGradDescent(arr& x, ScalarFunction& f, double initialStepSize, double *fmin_return, double stoppingTolerance, uint maxEvals, double maxStepSize, uint verbose ){
  uint evals=0;
  arr y, grad_x, grad_y;
  double fx, fy;
  double a=initialStepSize;

  fx = f.fs(&grad_x, x);  evals++;
  if(verbose>1) cout <<"*** optGradDescent: starting point x=" <<x <<" f(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(verbose>0) fil.open("z.grad");
  if(verbose>0) fil <<0 <<' ' <<fx <<' ' <<a <<endl;
  
  grad_x /= norm(grad_x);

  for(;;){
    y = x - a*grad_x;
    fy = f.fs(&grad_y, y);  evals++;
    CHECK(fy==fy, "cost seems to be NAN: fy=" <<fy);
    if(verbose>1) cout <<evals <<" \tprobing y=" <<y <<" \tf(y)=" <<fy <<" \t|grad|=" <<norm(grad_x) <<" \ta=" <<a;

    if(fy <= fx){
      if(verbose>1) cout <<" - ACCEPT" <<endl;
      double step=norm(x-y);
      x = y;
      fx = fy;
      grad_x = grad_y/norm(grad_y);
      a *= 1.2;
      if(maxStepSize>0. && a>maxStepSize) a = maxStepSize;
      if(verbose>0) fil <<evals <<' ' <<fx <<' ' <<a <<endl;
      if(step<stoppingTolerance) break;
    }else{
      if(verbose>1) cout <<" - reject" <<endl;
      a *= .5;
    }
    if(evals>maxEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.grad' us 1:2 w l");
  return evals;
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
    }else{                               //after change of direcion
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    }
  }
}

void Rprop::step(arr& x, ScalarFunction& f){
  arr grad;
  f.fs(&grad, x);
  step(x, grad);
}

//----- the rprop wrapped with stopping criteria
uint Rprop::loop(arr& _x,
                 ScalarFunction& f,
                 double *fmin_return,
                 double stoppingTolerance,
                 uint maxEvals,
                 uint verbose){
  arr x, J(_x.N), xmin;
  double y, ymin=0;
  uint lost_steps=0, small_steps=0;
  x=_x;
  
  ofstream fil;
  if(verbose>0) fil.open("z.rprop");
  
  uint i;
  for(i=0; i<maxEvals; i++){
    //checkGradient(p, x, stoppingTolerance);
    //compute value and gradient at x
    y = f.fs(&J, x);
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
    if(verbose>0) fil <<i <<' ' <<y <<' ' <<diff <<endl;
    if(diff<stoppingTolerance){ small_steps++; }else{ small_steps=0; }
    if(small_steps>10)  break;
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.rprop' us 1:2 w l");
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
