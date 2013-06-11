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



#ifndef MT_functions_h
#define MT_functions_h

#include <Core/array.h>

#define EXP ::exp //MT::approxExp

double NNsdv(const double& a, const double& b, double sdv);
double NNsdv(double x, double sdv);
double NNinv(const doubleA& a, const doubleA& b, const doubleA& Cinv);
double logNNprec(const arr& a, const arr& b, double prec);
double logNNinv(const doubleA& a, const doubleA& b, const doubleA& Cinv);
double NN(const doubleA& a, const doubleA& b, const doubleA& C);
double logNN(const doubleA& a, const doubleA& b, const doubleA& C);

/// non-normalized!! Gaussian function (f(0)=1)
double NNNNinv(const doubleA& a, const doubleA& b, const doubleA& Cinv);
double NNNN(const doubleA& a, const doubleA& b, const doubleA& C);
double NNzeroinv(const doubleA& x, const doubleA& Cinv);
/// gradient of a Gaussian
double dNNinv(const doubleA& x, const doubleA& a, const doubleA& Ainv, doubleA& grad);
/// gradient of a non-normalized Gaussian
double dNNNNinv(const doubleA& x, const doubleA& a, const doubleA& Ainv, doubleA& grad);
double NNsdv(const doubleA& a, const doubleA& b, double sdv);
double NNzerosdv(const doubleA& x, double sdv);

double smoothRamp(double x, double eps, double power);
double d_smoothRamp(double x, double eps, double power);

double barrier(double x, double margin, double power);
double d_barrier(double x, double margin, double power);

double potential(double x, double margin, double power);
double d_potential(double x, double margin, double power);

#ifdef  MT_IMPLEMENTATION
#  include "functions.cpp"
#endif

#endif
