/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
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


#ifndef _MT_motion_h
#define _MT_motion_h

#include <Ors/ors.h>
#include <Optim/optimization.h>

/* Notes
  -- transition models: kinematic, non-holonomic (vel = B u), pseudo dynamic, non-hol dynamic (acc = B u), real dynamic
  -- transition costs: vel^2 *tau, acc^2 * tau, u^2 * tau
  */


//===========================================================================
//
// defines only a map (task space), not yet the costs in this space
//

struct TaskMap {
  TermType type; // element of {cost_feature, inequality, equality} MAYBE: move this to Task?
  uint order;       ///< 0=position, 1=vel, etc
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G) = 0; ///< this needs to be overloaded
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau); ///< if not overloaded this computes the generic pos/vel/acc depending on order
  virtual uint dim_phi(const ors::KinematicWorld& G) = 0; //the dimensionality of $y$

  TaskMap():type(sumOfSqrTT),order(0) {}
  virtual ~TaskMap() {};
};


//===========================================================================
//
/// A k-order cost_feature, inequality or equality constraint,
/// optionally rescaled using 'target' and 'prec'
//

struct Task {
  TaskMap& map;
  MT::String name;
  bool active;
  arr target, prec;  ///< optional linear, potentially time-dependent, rescaling (with semantics of target & precision)

  uint dim_phi(const ors::KinematicWorld& G, uint t){
    if(!active || prec.N<=t || !prec(t)) return 0; return map.dim_phi(G); }

  Task(TaskMap* m):map(*m), active(true){}

  void setCostSpecs(uint fromTime, uint toTime,
                    const arr& _target=ARR(0.),
                    double _prec=1.);

};


//===========================================================================
//
// a motion problem description
//

/// This class allows you to DESCRIBE a motion planning problem, nothing more
struct MotionProblem {
  //engines to compute things
  ors::KinematicWorld& world;
  bool useSwift;
  
  //******* the following three sections are parameters that define the problem

  //-- task cost descriptions
  MT::Array<Task*> taskCosts;
  
  //-- trajectory length and tau
  uint T; ///< number of time steps
  double tau; ///< duration of single step
  
  //-- start constraints
  arr x0, v0; ///< fixed start state and velocity [[TODO: remove this and replace by prefix only (redundant...)]]
  arr prefix; ///< a set of states PRECEEDING x[0] (having 'negative' time indices) and which influence the control cost on x[0]. NOTE: x[0] is subject to optimization. DEFAULT: constantly equals x0
  arr postfix; ///< fixing the set of statex x[T-k]...x[T]
  //TODO: add methods to properly set the prefix given x0,v0?

  //-- stationary parameters
  arr z0; ///< an initialization of the stationary parameters of the motion problem

  //-- return values of an optimizer
  arrA phiMatrix;
  arr dualMatrix;
  MT::Array<TermTypeA> ttMatrix;

  MotionProblem(ors::KinematicWorld& _world, bool useSwift=true);
  
  MotionProblem& operator=(const MotionProblem& other);

  //-- setting costs in a task space
  Task* addTask(const char* name, TaskMap *map);
  //TODO: the following are deprecated; use Task::setCostSpecs instead
  enum TaskCostInterpolationType { constant, finalOnly, final_restConst, early_restConst, final_restLinInterpolated };
  void setInterpolatingCosts(Task *c,
                             TaskCostInterpolationType inType,
                             const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget=NoArr, double y_midPrec=-1., double earlyFraction=-1.);

  //-- cost infos
  bool getPhi(arr& phi, arr& J, TermTypeA& tt, uint t, const WorldL& G, double tau); ///< the general (`big') task vector and its Jacobian
  uint dim_phi(const ors::KinematicWorld& G, uint t);
  uint dim_g(const ors::KinematicWorld& G, uint t);
  uint dim_h(const ors::KinematicWorld& G, uint t);
  StringA getPhiNames(const ors::KinematicWorld& G, uint t);
  void costReport(bool gnuplt=true); ///< also computes the costMatrix
  
  void setState(const arr& x, const arr& v=NoArr);
  void activateAllTaskCosts(bool activate=true);

  //-- helpers
  arr getH_rate_diag();
  arr getInitialization();
};


//===========================================================================
//
// transforming a motion problem description into an optimization problem
//

struct MotionProblemFunction:KOrderMarkovFunction {
  MotionProblem& MP;
  WorldL configurations;

  MotionProblemFunction(MotionProblem& _P):MP(_P) { MT::Array<ors::KinematicWorld*>::memMove=true; };
  
  //KOrderMarkovFunction definitions
  virtual void phi_t(arr& phi, arr& J, TermTypeA& tt, uint t, const arr& x_bar);
  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() { return MP.T; }
  virtual uint get_k() { return 2; }
  virtual uint dim_x() { return MP.x0.N; }
  virtual uint dim_z() { return MP.z0.N; }
  virtual uint dim_phi(uint t){ return MP.dim_phi(MP.world, t); } //transitions plus costs (latter include constraints)
  virtual uint dim_g(uint t){ return MP.dim_g(MP.world, t); }
  virtual uint dim_h(uint t){ return MP.dim_h(MP.world, t); }
  virtual StringA getPhiNames(uint t);
  virtual arr get_prefix(); //the history states x(-k),..,x(-1)
  virtual arr get_postfix();
};


//===========================================================================
//
// transforming a motion problem description into an end-pose optimization problem only
//

struct MotionProblem_EndPoseFunction:VectorFunction {
  MotionProblem& MP;

  MotionProblem_EndPoseFunction(MotionProblem& _P):MP(_P) {};

  //VectorFunction definitions
  virtual void fv(arr& phi, arr& J, const arr& x);
};


//===========================================================================
//
// basic helpers
//

void sineProfile(arr& q, const arr& q0, const arr& qT,uint T);
arr reverseTrajectory(const arr& q);
void getVel(arr& v, const arr& q, double tau);
void getAcc(arr& a, const arr& q, double tau);


#endif
