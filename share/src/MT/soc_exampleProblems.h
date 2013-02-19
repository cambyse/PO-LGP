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

/**
 * @file
 * @ingroup group_soc
 */
/**
 * @addtogroup group_soc
 * @{
 */

#ifndef MT_soc_exampleProblems_h
#define MT_soc_exampleProblems_h

#include "socNew.h"

/**
 * An implementation of the SocSystemAbstraction that simulates a
 * single 1D point mass on a spring
 */
struct ControlledSystem_PointMass: ControlledSystem{

  uint T;
  double tau;
  arr x;
  arr x0, x_target;
  double prec;

  ControlledSystem_PointMass();
  virtual ~ControlledSystem_PointMass();

  // implementation of virtuals
  virtual uint get_T(){ return T; }
  virtual double get_tau(){ return tau; }
  virtual uint get_xDim(){ return 2; }
  virtual uint get_uDim(){ return 1; }
  virtual uint get_phiDim(uint t){ return get_xDim(); }
  virtual void get_x0(arr& x0){ x0 = this->x0; }
  virtual bool isKinematic(){ return false; }

  virtual void setx(const arr& x){ this->x = x; }
  virtual arr& getx(){ return this->x; }
  virtual void getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a, arr& B, arr& Bt, arr& Q, uint t);
  virtual void getControlCosts(arr& H, arr& Hinv, uint t);
  virtual void getTaskCosts(arr& phi, arr& phiJ, uint t);

  virtual void displayCurrentState(const char* title=NULL, bool pause=false, bool reportOnTasks=false);
  virtual void getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t);
};


#endif
/** @} */
