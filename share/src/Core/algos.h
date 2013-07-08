/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
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

/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef MT_algos_h
#define MT_algos_h

#include "array.h"

namespace MT {

//----- Runge-Kutta
/// standard Runge-Kutta 4
void rk4(arr& x1, const arr& x0,
         void (*df)(arr& xd, const arr& x),
         double dt);
/// same for second order diff equation
void rk4dd(arr& x1, arr& v1, const arr& x0, const arr& v0,
           void (*ddf)(arr& xdd, const arr& x, const arr& v),
           double dt);
/** RK with discrete event localization (zero-crossing detection): the
    function sf computes some double-valued indicators. If one of
    these indicators crosses zero this is interpreted as a
    discontinuity in the dynamics. The algorithm iteratively tries to
    find the zero-crossing point up to a tolerance tol (measured in
    time). The routine returns false for no-switching and true and the
    executed time step dt in the case of zero-crossing */
bool rk4_switch(arr& x1, arr& s1, const arr& x0, const arr& s0,
                void (*df)(arr& xd, const arr& x),
                void (*sf)(arr& s, const arr& x),
                double& dt, double tol);
/// same for 2nd order DEs
bool rk4dd_switch(arr& x1, arr& v1, arr& s1, const arr& x0, const arr& v0, const arr& s0,
                  void (*ddf)(arr& xdd, const arr& x, const arr& v),
                  void (*sf)(arr& s, const arr& x, const arr& v),
                  double& dt, double tol);
                  
/// a spline
struct Spline {
  uint T, K, degree;
  arr points;
  arr times;
  arr weights;
  arr basis, basis_trans, basis_timeGradient;

  Spline(uint T, arr& X, uint degree=2){ setUniformNonperiodicBasis(T, X.d0-1, degree); points=X; }
  void plotBasis();
  void setBasis();
  void setBasisAndTimeGradient();
  void setUniformNonperiodicBasis(uint T, uint K, uint degree);
  void eval(arr& f_t, uint t) const;
  void eval(arr& f) const;

  void partial(arr& dCdx, const arr& dCdf) const;
  void partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool constrain=true) const;
};

} //end namespace MT

#endif

/// @} //end group
