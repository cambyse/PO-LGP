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
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t=-1) = 0; ///< this needs to be overloaded
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau, int t=-1); ///< if not overloaded this computes the generic pos/vel/acc depending on order
  virtual uint dim_phi(const ors::KinematicWorld& G) = 0; //the dimensionality of $y$

  VectorFunction vf(ors::KinematicWorld& G){
    return [this, &G](arr& y, arr& J, const arr& x) -> void {
      G.setJointState(x);
      phi(y, J, G, -1);
    };
  }

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
  mlr::String name;
  bool active;
  arr target, prec;  ///< optional linear, potentially time-dependent, rescaling (with semantics of target & precision)

  uint dim_phi(const ors::KinematicWorld& G, uint t){
    if(!active || prec.N<=t || !prec(t)) return 0; return map.dim_phi(G); }

  Task(TaskMap* m):map(*m), active(true){} //TODO: require type here!!

  void setCostSpecs(uint fromTime, uint toTime,
                    const arr& _target=ARR(0.),
                    double _prec=1.);
};


Task* newTask(const Node* specs, const ors::KinematicWorld& world, uint Tinterval, uint Tzero=0);

//===========================================================================
//
// a motion problem description
//

/// This class allows you to DESCRIBE a motion planning problem, nothing more
struct MotionProblem {
  //engines to compute things
  ors::KinematicWorld& world;  ///< the original world
  WorldL configurations;       ///< copies for each time slice; including kinematic switches
  bool useSwift;
  
  //******* the following three sections are parameters that define the problem

  //-- task cost descriptions
  mlr::Array<Task*> tasks;

  //-- kinematic switches along the motion
  mlr::Array<ors::KinematicSwitch*> switches;

  //-- trajectory length and tau
  uint T; ///< number of time steps
  double tau; ///< duration of single step
  uint k_order; ///< determine the order of the KOMO problem (default 2)
  
  //-- start constraints
  arr x0;      ///< fixed start state and velocity [[TODO: remove this and replace by prefix only (redundant...)]]
  arr prefix;  ///< a set of states PRECEEDING x[0] (having 'negative' time indices) and which influence the control cost on x[0]. NOTE: x[0] is subject to optimization. DEFAULT: constantly equals x0
  arr postfix; ///< fixing the set of statex x[T-k]...x[T] //TODO: remove?
  //TODO: add methods to properly set the prefix given x0,v0?

  //-- return values of an optimizer
  arrA phiMatrix;
  arr dualMatrix;
  mlr::Array<TermTypeA> ttMatrix;

  MotionProblem(ors::KinematicWorld& _world, bool useSwift=true);
  
  MotionProblem& operator=(const MotionProblem& other);

  //-- setting time aspects
  void setTiming(uint timeSteps, double duration);

  //-- setting costs in a task space
  bool parseTask(const Node *n, int Tinterval=-1, uint Tzero=0);
  void parseTasks(const Graph& specs, int Tinterval=-1, uint Tzero=0);
  Task* addTask(const char* name, TaskMap *map);
  //TODO: the following are deprecated; use Task::setCostSpecs instead
//  enum TaskCostInterpolationType { constant, finalOnly, final_restConst, early_restConst, final_restLinInterpolated };
//  void setInterpolatingCosts(Task *c,
//                             TaskCostInterpolationType inType,
//                             const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget=NoArr, double y_midPrec=-1., double earlyFraction=-1.);

  //-- cost infos
  bool getPhi(arr& phi, arr& J, TermTypeA& tt, uint t, const WorldL& G, double tau); ///< the general task vector and its Jacobian
  uint dim_phi(uint t);
  uint dim_g(uint t);
  uint dim_h(uint t);
  StringA getPhiNames(uint t);
  void reportFull(bool brief=false);
  void costReport(bool gnuplt=true); ///< also computes the costMatrix
  Graph getReport();

  void setState(const arr& x, const arr& v=NoArr);
  void activateAllTaskCosts(bool activate=true);

  //-- helpers
  arr getH_rate_diag();
  arr getInitialization();
  void setConfigurationStates(const arr& x);
  void setupConfigurations();
  void temporallyAlignKinematicSwitchesInConfiguration(uint t);
  void displayTrajectory(int steps, const char *tag, double delay=0.);

  //-- inverse Kinematics
  void inverseKinematics(arr& y, arr& J, arr& H, TermTypeA& tt, const arr& x);

  ConstrainedProblem InvKinProblem(){
    return [this](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void {
      this->inverseKinematics(phi, J, H, tt, x);
    };
  }

//  KOrderMarkovFunction PathProblem(){
//    NIY;
//  }
};


//===========================================================================
//
// transforming a motion problem description into an optimization problem
//

struct MotionProblemFunction:KOrderMarkovFunction {
  MotionProblem& MP;

  MotionProblemFunction(MotionProblem& _P):MP(_P) {}

  uint dim_g_h(){ uint d=0; for(uint t=0;t<=MP.T;t++) d += dim_g(t) + dim_h(t); return d; }

  //KOrderMarkovFunction definitions
  virtual void set_x(const arr& x){ MP.setConfigurationStates(x); }
  virtual void phi_t(arr& phi, arr& J, TermTypeA& tt, uint t);
  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() { return MP.T; }
  virtual uint get_k() { return MP.k_order; }
  virtual uint dim_x() { uint d=0; for(uint t=0; t<=MP.T; t++) d+=dim_x(t); return d; }
  virtual uint dim_x(uint t) { return MP.configurations(t)->getJointStateDimension(); }
  virtual uint dim_phi(uint t){ return MP.dim_phi(t); } //transitions plus costs (latter include constraints)
  virtual uint dim_g(uint t){ return MP.dim_g(t); }
  virtual uint dim_h(uint t){ return MP.dim_h(t); }
  virtual StringA getPhiNames(uint t);
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
