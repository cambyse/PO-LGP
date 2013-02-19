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

//===========================================================================
#include "soc.h"
#include "ors.h"

//===========================================================================
namespace soc {
/**
 * An implementation of the SocSystemAbstraction using the \ref ors simulator.
 * @addtogroup group_soc
 * @{
 */
struct SocSystem_Ors: public virtual SocSystemAbstraction {
  ors::Graph *ors;
  SwiftInterface *swift;
  MT::Array<TaskVariable*> vars;
  struct sSocSystem_Ors *s;

  SocSystem_Ors();
  virtual ~SocSystem_Ors();
  SocSystem_Ors* newClone(bool deep) const;

  /// @name initialization methods
  void initBasics(ors::Graph *_ors, SwiftInterface *_swift, OpenGL *_gl,
                  uint trajectory_steps, double trajectory_time, bool _dynamic, arr *W);
  void setTimeInterval(double trajectory_time, uint trajectory_steps);
  void setTaskVariables(const TaskVariableList& CVlist);

  /// @name exemplary problem setups: read specifications from MT.cfg
  void initStandardReachProblem(uint rand_seed=0, uint T=0, bool _dynamic=false);
  void initStandardBenchmark(uint rand_seed=0);

  /// @name info
  void reportOnState(std::ostream& os);
  void displayState(const arr *x, const arr *Q=NULL, const char *text=NULL, bool reportVariables=false);
  void recordTrajectory(const arr& q,const char *variable,const char *file);

  /// @name implementations of virtual methods
  uint get_T();
  uint get_xDim();
  uint nTasks();
  uint qDim();
  uint uDim();
  uint yDim(uint i);
  void getq0(arr& q);
  void setq0(const arr& q0);
  void getv0(arr& v);
  void getx0(arr& x);
  void setx0(const arr& x0);
  void getqv0(arr& q, arr& qd);
  bool isDynamic();
  void setq(const arr& q, uint t=0);
  void setx(const arr& x, uint t=0);
  void setqv(const arr& q, const arr& qd, uint t=0);
  void setx0ToCurrent();
  //void geth  (arr& h);
  virtual void getControlCosts(arr& H, arr& Hinv, uint t);
  void getHrateInv(arr& HrateInv);
  bool isConditioned(uint i, uint t);
  bool isConstrained(uint i, uint t);
  const char* taskName(uint i);
  uint taskDim(uint i);
  void getPhi(arr& phiq_i, uint i);
  void getJJt(arr& J_i, arr& Jt_i, uint i);
  void getHessian(arr& H_i, uint i);
  void getJqd(arr& Jqd_i, uint i);
  void getTarget(arr& y_i, double& prec, uint i, uint t);
  void getTargetV(arr& v_i, double& prec, uint i, uint t);
  //void getC   (arr& C_i, uint i, uint t);
  //void getCV  (arr& D_i, uint i, uint t);
  double getTau(bool scaled=true);
  void getMF(arr& M, arr& F, arr& Q, uint t);
  void getMinvF(arr& Minv, arr& F, arr& Q, uint t);
  virtual void setTau(double tau);
};

/** @} */
}
/** @} */ // END of namespace group

#endif
