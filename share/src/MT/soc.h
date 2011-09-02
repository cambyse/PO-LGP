/*  Copyright 2009 Marc Toussaint
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
    along with this program. If not, see <http://www.gnu.org/licenses/> */

/** \file soc.h
    \brief Stochastic Optimal Control library */

#ifndef MT_soc_h
#define MT_soc_h

#include "util.h"
#include "array.h"

//-- fwd declarations
class OpenGL;
struct SwiftInterface;
namespace ors { struct Graph; }
struct TaskVariable;
typedef MT::Array<TaskVariable*> TaskVariableList;
extern uint countMsg, countSetq;


namespace soc {

/// gradient method options
//enum SocSolverType{ ConjGrad=0, LevMar=1, Rprop=2, RpropConjGrad=3, SQP=4, Attractor=5 };
enum { ConjGrad=0, LevMar=1, Rprop=2, RpropConjGrad=3, SQP=4, Attractor=5 };


//===========================================================================
//
// SocSystemAbstraction
//

/** \brief defines an abstraction of stochastic optimal control
    problems which interfaces between solution methods and system simulators
    -- see section 3.2 of the <a href="../guide.pdf">guide</a> */
struct SocSystemAbstraction {

  ///@name data fields
  std::ostream *os; ///< if non-NULL, some routines might give output
  OpenGL *gl;       ///< if non-NULL, some routines might give output
  bool dynamic;     ///< determines whether this problem is dynamic or not
  uint scalePower;  ///< if non-zero, all routines assume an horizon T=T/2^scalePower
  
  uintA stepScale;    ///< the scale of each step (time interval between i-th and (i+1)-th step=2^scale)
  
  ///@name initialization
  SocSystemAbstraction();
  virtual ~SocSystemAbstraction();
  virtual SocSystemAbstraction *newClone() const; ///< creates a new clone of this SocAbstraction (deep copy of simulators etc)
  
  ///@name low level access routines: need to be implemented by the simulator
  
  // access general problem information
  virtual uint nTime() = 0;            ///< total time steps of the trajectory
  virtual uint nTasks() = 0;           ///< number of task variables
  virtual uint qDim() = 0;             ///< dimensionality of q-space
  virtual uint uDim();                 ///< dimensionality of control
  virtual uint yDim(uint i) = 0;       ///< dimensionality of the i-th task
  virtual void getq0(arr& q) = 0;      ///< start joint configuration
  virtual void getv0(arr& v) = 0;      ///< start joint velocity
  virtual void getqv0(arr& q_);        ///< start joint configuration and velocity
  virtual void getqv0(arr& q, arr& qd); ///< start joint configuration and velocity
  virtual double getTau(bool scaled=true);    ///< time step size (for dynamic problems)
  void getx0(arr& x){ if(dynamic) getqv0(x); else getq0(x); }
  
  // set x-state (following calls to getPhi and getJ are w.r.t. this x)
  virtual void setq(const arr& q, uint t=0) = 0;
  virtual void setqv(const arr& q_, uint t=0);
  virtual void setqv(const arr& q, const arr& qd, uint t=0);
  void setx(const arr& x){ if(dynamic) setqv(x); else setq(x); }
  virtual void setq0AsCurrent() = 0;
  virtual void setToq0(){ arr q; getq0(q); setq(q); }
  
  //motion prior, or control cost  [t indicates the step]
  virtual void getW(arr& W, uint t) = 0;          ///< kinematic step cost metric: cost = dq^T W dq
  virtual void getWinv(arr& Winv, uint t){ throw("NIY"); } ///< kinematic step cost metric: cost = dq^T W dq
  virtual void getH(arr& H, uint t);              ///< dynamic control cost metric: cost = u^T H u
  virtual void getHinv(arr& H, uint t);           ///< dynamic control cost metric: cost = u^T H u
  virtual void getQ(arr& Q, uint t);              ///< process stochasticity or integration noise Q (e.g., setDiag(1e-10, qDim()) )
  
  // dynamic model
  virtual void getMF(arr& M, arr& F, uint t);
  virtual void getMinvF(arr& Minv, arr& F, uint t);
  
  // task coupling
  virtual bool isConditioned(uint i, uint t) = 0;
  virtual bool isConstrained(uint i, uint t);
  virtual const char* taskName(uint i){ return NULL; };
  virtual void getPhi(arr& phiq_i, uint i){ throw("NIY"); }
  virtual void getJJt(arr& J_i, arr& Jt_i, uint i){ throw("NIY"); }
  virtual void getJqd(arr& Jqd_i, uint i);
  virtual void getHessian(arr& H_i, uint i);
  virtual void getTarget(arr& y_i, double& prec, uint i, uint t){ throw("NIY"); }
  virtual void getTargetV(arr& v_i, double& prec, uint i, uint t);
  //virtual void getLinearConstraint(arr& c, double& coff, uint i, uint t); ///< defines a cost 1 iff [[c^T y_i + coff>0]]
  
  
  
  ///@name high level methods: they are being accessed by the solvers
  
  // abstract SOC interface
  virtual void getTaskCostTerms(arr& Phi, arr& PhiJ, const arr& xt, uint t); ///< the general (`big') task vector and its Jacobian
  virtual void getTransitionCostTerms(arr& Psi, arr& PsiI, arr& PsiJ, const arr& xt_1, const arr& xt, uint t);
  virtual void getProcess(arr& A, arr& a, arr& B, uint t, arr* Winv=NULL);
  virtual void getProcess(arr& A, arr& tA, arr& Ainv, arr& invtA, arr& a, arr& B, arr& tB, uint t);
  virtual double getCosts(arr& R, arr& r, const arr& qt, uint t, double* rhat=NULL);
  virtual void getConstraints(arr& c, arr& coff, const arr& qt, uint t);
  
  // cost info
  double taskCost(arr* grad, int t, int whichTask, bool verbose=false); //whichTask=-1 -> all, verbose: print individual task costs
  double totalCost(arr *grad, const arr& q, bool plot=false);
  
  virtual void displayState(const arr *q, const arr *Qinv, const char *text=NULL, bool reportVariables=false);
  virtual void displayTrajectory(const arr& q, const arr *Qinv, int steps, const char *tag);
  
  //-- convenience (prelim...)
  void costChecks(const arr& x); //computes the costs in many different ways - check if they're equal - code is instructive...
  double analyzeTrajectory(const arr& q, bool plot);
  void constantTrajectory(arr& q);
  void passiveDynamicTrajectory(arr& q);
  void controlledDynamicTrajectory(arr& q, const arr& u);
  void getControlFromTrajectory(arr& u, const arr& q);
};


//===========================================================================
//
///@name     trivial helpers
// @{

void getVelocity(arr& vt, const arr& q, uint t, double tau);
void getPhaseTrajectory(arr& _q, const arr& q, double tau);
void getPositionTrajectory(arr& q, const arr& _q);
void interpolateTrajectory(arr& qNew, const arr& _q, double step);

//only for the first task so far!
void getJointFromTaskTrajectory(SocSystemAbstraction& soci, arr& q, const arr& x);
void partialJointFromTaskTrajectory(SocSystemAbstraction& soci, arr& dx, const arr& dq, const arr& q, const arr& x);
void straightTaskTrajectory(SocSystemAbstraction& soci, arr& q, uint taskid);


} //namespace

//===========================================================================
//
// implementations
//

#ifdef  MT_IMPLEMENTATION
#  include "soc.cpp"
#endif

#endif
