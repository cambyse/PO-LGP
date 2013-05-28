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
#ifndef MT_soc_h
#define MT_soc_h

/**
 * @file
 * @ingroup group_soc
 */

#include "util.h"
#include "array.h"
#include "optimization.h"

//-- fwd declarations
struct OpenGL;
extern uint countMsg, countSetq;


/**
 * @addtogroup group_soc
 * @{
 */
namespace soc {
/**
 * @addtogroup group_soc
 * @{
 */

//===========================================================================
/**
 * Defines an abstraction of stochastic optimal control
 * problems which interfaces between solution methods and system simulators.
 */
struct SocSystemAbstraction:VectorChainFunction {

  /// @name data fields
  std::ostream *os; ///< if non-NULL, some routines might give output
  OpenGL *gl;       ///< if non-NULL, some routines might give output
  bool dynamic;     ///< determines whether this problem is dynamic or not
  uint scalePower;  ///< if non-zero, all routines assume an horizon T=T/2^scalePower

  uintA stepScale;   ///< the scale of each step (time interval between i-th and (i+1)-th step=2^scale)
  float checkGrad;   ///<the probability by which the gradients are checked in each call of getTaskCost[Terms]

  /// @name initialization
  SocSystemAbstraction();
  virtual ~SocSystemAbstraction();
  virtual SocSystemAbstraction *newClone() const; ///< creates a new clone of this SocAbstraction (deep copy of simulators etc)

  /// @name low level access routines: need to be implemented by the simulator

  // access general problem information
  bool isKinematic(){ return !dynamic; }
  virtual uint get_T() = 0;                 ///< total time steps of the trajectory (that's time_slices - 1)
  virtual uint get_xDim() = 0;              ///< total time steps of the trajectory (that's time_slices - 1)
  virtual uint nTasks() = 0;                ///< number of task variables
  virtual uint qDim() = 0;                  ///< dimensionality of q-space
  virtual uint uDim();                      ///< dimensionality of control
  virtual uint yDim(uint i) = 0;            ///< dimensionality of the i-th task
  virtual void getq0(arr& q0) = 0;          ///< start joint configuration
  virtual void getv0(arr& v0) = 0;          ///< start joint velocity
  virtual void getx0(arr& x0);              ///< start joint configuration and velocity
  virtual void getqv0(arr& q0, arr& v0);    ///< start joint configuration and velocity
  virtual double getTau(bool scaled=true);  ///< time step size (for dynamic problems)
  virtual void setTau(double tau) = 0;
  virtual double getDuration(){ return getTau()*get_T(); }

  // set x-state (following calls to getPhi and getJ are w.r.t. this x)
  virtual void setx0ToCurrent() = 0;
  virtual void setTox0(){ arr x; getx0(x); setx(x); }
  virtual void setq(const arr& q, uint t=0) = 0;
  virtual void setx(const arr& x, uint t=0) = 0;
  virtual void setqv(const arr& q, const arr& qd, uint t=0){ setx(cat(q,qd),t); }

  //motion prior, or control cost PER STEP unless rate is explicitly indicated [t indicates the step]
  virtual void getControlCosts(arr& H, arr& Hinv, uint t);              ///< dynamic control cost metric: step cost = u^T H u, with H = tau*H_rate where tau is step size

  // dynamic model
  virtual void getMF(arr& M, arr& F, arr& Q, uint t);
  virtual void getMinvF(arr& Minv, arr& F, arr& Q, uint t);

  // task coupling
  virtual bool isConditioned(uint i, uint t) = 0;
  virtual bool isConstrained(uint i, uint t);
  virtual const char* taskName(uint i){ return NULL; };
  virtual uint taskDim(uint i){ return 0; };
  virtual void getPhi(arr& phiq_i, uint i){ throw("NIY"); }
  virtual void getJJt(arr& J_i, arr& Jt_i, uint i){ throw("NIY"); }
  virtual void getJqd(arr& Jqd_i, uint i);
  virtual void getHessian(arr& H_i, uint i);
  virtual void getTarget(arr& y_i, double& prec, uint i, uint t){ throw("NIY"); }
  virtual void getTargetV(arr& v_i, double& prec, uint i, uint t);
  //virtual void getLinearConstraint(arr& c, double& coff, uint i, uint t); ///< defines a cost 1 iff [[c^T y_i + coff>0]]


  /// @name high level methods: they are being accessed by the solvers

  // abstract SOC interface
  virtual void getTaskCostTerms(arr& Phi, arr& PhiJ, const arr& xt, uint t); ///< the general (`big') task vector and its Jacobian
  virtual void getTransitionCostTerms(arr& Psi, arr& PsiI, arr& PsiJ, const arr& xt, const arr& xt1, uint t);
  virtual void getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t, arr* Winv=NULL);
  virtual void getDynamics(arr& A, arr& tA, arr& Ainv, arr& invtA, arr& a, arr& B, arr& tB, arr& Q, uint t);
  virtual double getTaskCosts(arr& R, arr& r, const arr& qt, uint t, double* rhat=NULL);
  virtual void getConstraints(arr& c, arr& coff, const arr& qt, uint t);
  void getTaskInfo(MT::Array<const char*>& names, uintA& dims, uint t);

  //old cost computation routines -- still very useful for reference and checking
  double taskCost(arr* grad, int t, int whichTask, bool verbose=false); //whichTask=-1 -> all, verbose: print individual task costs
  double totalCost(arr *grad, const arr& q, bool plot=false);
  void costChecks(const arr& x); //computes the costs in many different ways - check if they're equal - code is instructive...

  //VectorChainFunction optimization interface
  virtual void fv_i (arr& y, arr& J, uint i, const arr& x_i);
  virtual void fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j);

  /// @name utilities:

  virtual void displayState(const arr *q, const arr *Qinv=NULL, const char *text=NULL, bool reportVariables=false);
  virtual void recordTrajectory(const arr& q,const char *variable,const char *file);
  virtual void displayTrajectory(const arr& q, const arr *Qinv, int steps, const char *tag);

  //-- convenience (prelim...)
  void testGradientsInCurrentState(const arr& xt, uint t);
  double analyzeTrajectory(const arr& q, bool plot);
  void constantTrajectory(arr& q);
  void passiveDynamicTrajectory(arr& q);
  void controlledDynamicTrajectory(arr& q, const arr& u);
  void getControlFromTrajectory(arr& u, const arr& q);
};


//===========================================================================
/// @name     trivial helpers
/// @{

void getVelocity(arr& vt, const arr& q, uint t, double tau);
void getPhaseTrajectory(arr& x, const arr& q, double tau);
void getPositionTrajectory(arr& q, const arr& x);
//void interpolateTrajectory(arr& qNew, const arr& _q, double step);

//only for the first task so far!
void straightTaskTrajectory(SocSystemAbstraction& soci, arr& q, uint taskid);
/// @}

//===========================================================================
} //namespace
/** @} */  // END group group_soc


//===========================================================================
//
// implementations
//
#ifdef  MT_IMPLEMENTATION
#  include "soc.cpp"
#endif

#endif
