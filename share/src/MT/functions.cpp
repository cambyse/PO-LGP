/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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



#include "functions.h"
#include <Algo/algos.h>

double NNsdv(const double& a, const double& b, double sdv){
  double d=(a-b)/sdv;
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*EXP(-.5*d*d);
}
double NNsdv(double x, double sdv){
  x/=sdv;
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*EXP(-.5*x*x);
}
double NNinv(const arr& a, const arr& b, const arr& Cinv){
  double d=sqrDistance(Cinv, a, b);
  double norm = ::sqrt(lapack_determinantSymPosDef((1./MT_2PI)*Cinv));
  return norm*EXP(-.5*d);
}
double logNNinv(const arr& a, const arr& b, const arr& Cinv){
  NIY;
  return 1;
  /*
  arr d=a-b;
  double norm = ::sqrt(fabs(MT::determinant_LU((1./MT_2PI)*Cinv)));
  return ::log(norm) + (-.5*scalarProduct(Cinv, d, d));
  */
}
double logNNprec(const arr& a, const arr& b, double prec){
  uint n=a.N;
  arr d=a-b;
  double norm = pow(prec/MT_2PI, .5*n);
  return ::log(norm) + (-.5*prec*scalarProduct(d, d));
}
double logNN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return logNNinv(a, b, Cinv);
}
double NN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return NNinv(a, b, Cinv);
}
/// non-normalized!! Gaussian function (f(0)=1)
double NNNNinv(const arr& a, const arr& b, const arr& Cinv){
  double d=sqrDistance(Cinv, a, b);
  return EXP(-.5*d);
}
double NNNN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return NNNNinv(a, b, Cinv);
}
double NNzeroinv(const arr& x, const arr& Cinv){
  double norm = ::sqrt(lapack_determinantSymPosDef((1./MT_2PI)*Cinv));
  return norm*EXP(-.5*scalarProduct(Cinv, x, x));
}
/// gradient of a Gaussian
double dNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad){
  double y=NNinv(x, a, Ainv);
  grad = y * Ainv * (a-x);
  return y;
}
/// gradient of a non-normalized Gaussian
double dNNNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad){
  double y=NNNNinv(x, a, Ainv);
  grad = y * Ainv * (a-x);
  return y;
}
double NNsdv(const arr& a, const arr& b, double sdv){
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*EXP(-.5*sqrDistance(a, b)/(sdv*sdv));
}
double NNzerosdv(const arr& x, double sdv){
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*EXP(-.5*sumOfSqr(x)/(sdv*sdv));
}

/* gnuplot:
heavy(x) = (1+sgn(x))/2
eps = 0.1
g(x) = heavy(x-eps)*(x-eps/2) + (1-heavy(x-eps))*x**2/(2*eps)
plot [-.5:.5] g(abs(x))
*/
double POW(double x, double power){ if(power==1.) return x; if(power==2.) return x*x; return pow(x,power); }
double smoothRamp(double x, double eps, double power){
  if(x<0.) return 0.;
  if(power!=1.) return pow(smoothRamp(x,eps,1.),power);
  if(!eps) return x;
  if(x>eps) return x - .5*eps;
  return x*x/(2*eps);
}

double d_smoothRamp(double x, double eps, double power){
  if(x<0.) return 0.;
  if(power!=1.) return power*pow(smoothRamp(x,eps,1.),power-1.)*d_smoothRamp(x,eps,1.);
  if(!eps || x>eps) return 1.;
  return x/eps;
}

/*
heavy(x) = (1+sgn(x))/2
power = 1.5
margin = 1.5
f(x) = heavy(x)*x**power
plot f(x/margin+1), 1
*/
double barrier(double x, double margin, double power){
  if(x<-margin) return 0.;
  double y=x/margin+1.;
  if(power==1.) return y;
  if(power==2.) return y*y;
  return pow(y,power);
}

double d_barrier(double x, double margin, double power){
  if(x<-margin) return 0.;
  double y=x/margin+1.;
  if(power==1.) return 1./margin;
  if(power==2.) return 2.*y/margin;
  return power*pow(y,power-1.)/margin;
}

double potential(double x, double margin, double power){
  double y=x/margin;
  if(power==1.) return fabs(y);
  if(power==2.) return y*y;
  return pow(fabs(y),power);
}

double d_potential(double x, double margin, double power){
  double y=x/margin;
  if(power==1.) return MT::sign(y)/margin;
  if(power==2.) return 2.*y/margin;
  return power*pow(y,power-1.)*MT::sign(y)/margin;
}


