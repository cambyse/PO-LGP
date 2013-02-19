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
#ifndef MT_socSystem_ors_h
#define MT_socSystem_ors_h
/**
 * @file
 * @ingroup group_soc
 */
/**
 * @addtogroup group_soc
 * @{
 */

#include "socNew.h"
#include "ors.h"

//===========================================================================
/**
 * an implementation of the SocSystemAbstraction using the \ref ors
 * simulator.
 */
struct OrsSystem: ControlledSystem {
  struct sOrsSystem *s;

  OrsSystem();
  virtual ~OrsSystem();
  OrsSystem* newClone(bool deep) const;

  /// @name initialization methods
  void initBasics(ors::Graph *_ors, SwiftInterface *_swift, OpenGL *_gl,
                  uint trajectory_steps, double trajectory_time, bool _dynamic, arr *W);
  /// @name exemplary problem setups: read specifications from MT.cfg
  void initStandardReachProblem(uint rand_seed=0, uint T=0, bool _dynamic=false);
  void initStandardBenchmark(uint rand_seed=0);

  /// @name junk
  void setTau(double tau);
  void setTimeInterval(double trajectory_time, uint trajectory_steps);
  void setTaskVariables(const TaskVariableList& CVlist);
  void setx0ToCurrent();
  void setTox0();
  void setx0(const arr& x0);
  uint get_qDim();
  ors::Graph& getOrs();
  SwiftInterface& getSwift();
  MT::Array<TaskVariable*>& vars();

  /// @name implementations of virtual methods
  uint get_T();
  double get_tau();
  uint get_xDim();
  uint get_uDim();
  uint get_phiDim(uint t);
  void get_x0(arr& x0);
  bool isKinematic();
  void setx(const arr& x);
  arr& getx();
  void getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t);
  void getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a, arr& B, arr& Bt, arr& Q, uint t);
  void getControlCosts(arr& H, arr& Hinv, uint t);
  void getTaskCosts(arr& phi, arr& phiJ, uint t);
  void displayCurrentState(const char* title=NULL, bool pause=false, bool reportOnTasks=false);
  void getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t);
};

/** @} */
#endif
