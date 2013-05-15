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

#include "optimization.h"

//===========================================================================

struct RosenbrockFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    if(&H) NIY;
    double f=0.;
    for(uint i=1;i<x.N;i++) f += MT::sqr(x(i)-MT::sqr(x(i-1))) + .01*MT::sqr(1-x(i-1));
    if(&g) NIY;
    return f;
  }
};

//===========================================================================

struct RastriginFunction:ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) {
    if(&H) NIY;
    double A=.5, f=A*x.N;
    for(uint i=0;i<x.N;i++) f += x(i)*x(i) - A*::cos(10.*x(i));
    if(&g){
      g.resize(x.N);
      for(uint i=0;i<x.N;i++) g(i) = 2*x(i) + 10.*A*::sin(10.*x(i));
    }
    if(&H){
      H.resize(x.N,x.N);  H.setZero();
      for(uint i=0;i<x.N;i++) H(i,i) = 2 + 100.*A*::cos(10.*x(i));
    }
    return f;
  }
};

//===========================================================================

struct SquareFunction:ScalarFunction{
  virtual double fs(arr& g, arr& H, const arr& x) {
    double f=sumOfSqr(x);
    if(&g) g=2.*x;
    if(&H) H.setId(x.N);
    return f;
  }
};

//===========================================================================

struct HoleFunction:ScalarFunction{
  virtual double fs(arr& g, arr& H, const arr& x) {
    double f=sumOfSqr(x);
    if(&g) g=exp(-f)*2.*x;
    if(&H){ H.setId(x.N); H *= -2*exp(-f); }
    f = 1.-exp(-f);
    return f;
  }
};

//===========================================================================

struct ChoiceFunction:ScalarFunction{
  enum Which{ none=0, square, hole, rosenbrock, rastrigin } which;
  double condition;
  ChoiceFunction(){
    which = (Which) MT::getParameter<int>("fctChoice");
    condition = MT::getParameter<double>("condition");
  }
  virtual double fs(arr& g, arr& H, const arr& x) {
    double f;
    arr c(x.N), y(x);
    for(uint i=0;i<c.N;i++) c(i) = pow(condition,(double)i/(x.N-1));
    y *= c; //elem-wise product
    switch(which){
    case square: f = SquareFunction().fs(g, H, y); break;
    case hole: f = HoleFunction().fs(g, H, y); break;
    default: NIY;
    }
    if(&g) g*=c; //elem-wise product
    if(&H) NIY;
    return f;
  }
};

//===========================================================================

struct ChoiceConstraintFunction:VectorFunction{
  ChoiceFunction f;
//  double margin;
  ChoiceConstraintFunction(){
//    margin = MT::getParameter<double>("constraintMargin");
  }
  virtual void fv(arr& phi, arr& J, const arr& x) {
    uint n=x.N;
    arr J_f;
    phi.resize(2);
    if(&J){ J.resize(2, n); J.setZero(); }
    phi(0) = f.fs( (&J?J[0]():NoArr), NoArr, x);
    phi(1) = x(0) + .5;
    if(&J) J(1,0) = 1.;
  }
};

//===========================================================================

/// $f(x) = x^T C x$ where C has eigen values ranging from 1 to 'condition'
struct SquaredCost:public ScalarFunction,VectorFunction {
  arr M,C; /// $C = M^T M $
  uint n;  /// dimensionality of $x$
  
  SquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);
  
  double fs(arr& g, arr& H, const arr& x);
  void fv(arr& y, arr& J,const arr& x);
};

//===========================================================================

/// Same as SquaredCost but $x_i \gets atan(x_i)$ before evaluating the squared cost
struct NonlinearlyWarpedSquaredCost:public ScalarFunction,VectorFunction {
  uint n;  /// dimensionality of $x$
  SquaredCost sq;
  
  NonlinearlyWarpedSquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);
  
  double fs(arr& grad, arr& H, const arr& x);
  void fv(arr& y, arr& J,const arr& x);
};

//===========================================================================

struct VectorChainCost:VectorChainFunction {
  uint T,n;
  arr A,a;
  arr Wi,Wj,w;
  bool nonlinear;
  
  VectorChainCost(uint _T,uint _n);
  uint get_T(){ return T; }
  void fv_i(arr& y, arr* J, uint i, const arr& x_i);
  void fv_ij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j);
};

//===========================================================================

struct SlalomProblem:VectorChainFunction {
  uint T,K,n;
  double margin,w,power;
  
  SlalomProblem(uint _T, uint _K, double _margin, double _w, double _power);
  uint get_T(){ return T; }
  void fv_i(arr& y, arr& J, uint i, const arr& x_i);
  void fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j);
};
