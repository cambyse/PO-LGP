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
#ifndef MT_socNew_h
#define MT_socNew_h
/**
 * @file
 * @ingroup group_soc
 */
/**
 * @addtogroup group_soc
 * @{
 */

#include "util.h"
#include "array.h"
#include "optimization.h"

extern uint countMsg, countSetq;


//===========================================================================
//
// ControlledSystem
//
struct KOrderMarkovFunction_ControlledSystem;

struct ControlledSystem {
  ControlledSystem():os(&std::cout), gl(NULL) {}

  /// @name access general problem information
  virtual uint get_T() = 0;            ///< total time steps of the trajectory (that's time_slices - 1)
  virtual double get_tau() = 0;
  virtual uint get_xDim() = 0;         ///< the dimensionality of \f$x_t\f$
  virtual uint get_uDim() = 0;         ///< dimensionality of control
  virtual uint get_phiDim(uint t) = 0; ///< the dimensionality of the task vector \f$\phi_t\f$
  virtual void get_x0(arr& x0) = 0;    ///< start joint configuration and velo
  virtual bool isKinematic() = 0;

  /// @name access dynamics and task vector for given state x.
  /// NOTE: arguments may be @NoArr@ if they're not needed!!
  virtual void setx(const arr& x) = 0;
  virtual arr& getx() = 0;             ///< get the current x!
  virtual void getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a,
                           arr& B, arr& Bt, arr& Q, uint t) = 0;
  virtual void getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t){ getDynamics(A, NoArr, NoArr, NoArr, a, B, NoArr, Q, t); }
  virtual void getControlCosts(arr& H, arr& Hinv, uint t) = 0; ///< dynamic control cost metric: step cost = u^T H u, with H = tau*H_rate where tau is step size
  virtual void getTaskCosts(arr& phi, arr& phiJ, uint t) = 0;  ///< the general vector and its Jacobian
  virtual double getTaskCosts(arr& R, arr& r, uint t, double* rhat); /// @todo REMOVE THIS!

  /// @name display and info
  virtual void displayCurrentState(const char* title=NULL, bool pause=false, bool reportOnTasks=false) = 0;
  virtual void getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t) = 0;

  std::ostream *os;
  struct OpenGL *gl;
};


//===========================================================================
/// @name helpers to analyze and handle trajectories
/// @todo ->move to soc_helpers
void getTransitionCostTerms(ControlledSystem& sys, bool dynamic, arr& Psi, arr& J0, arr& J1, const arr& x0, const arr& x1, uint t);
double analyzeTrajectory(ControlledSystem& sys, const arr& x, bool plot, std::ostream* os);
void displayTrajectory(ControlledSystem& sys, const arr& x, const arr *Binv, int steps, const char *tag);
void getVelocity(arr& vt, const arr& q, uint t, double tau);
void getPhaseTrajectory(arr& x, const arr& q, double tau);
void getPositionTrajectory(arr& q, const arr& x);
void straightTaskTrajectory(ControlledSystem& sys, arr& q);


//===========================================================================
/// @name basic control
void dynamicControl(ControlledSystem& sys, arr& x, const arr& x0, uint t, arr *v=NULL, arr *Vinv=NULL);


/** @} */
#endif
