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


#pragma once

#include <Ors/ors.h>
#include <Optim/optimization.h>
#include "taskMap.h"

/* Notes
  -- transition models: kinematic, non-holonomic (vel = B u), pseudo dynamic, non-hol dynamic (acc = B u), real dynamic
  -- transition costs: vel^2 *tau, acc^2 * tau, u^2 * tau
  */

//===========================================================================
//
/// A k-order sumOfSqr feature, inequality or equality constraint,
/// optionally rescaled using 'target' and 'prec'
//

struct Task {
  TaskMap& map;
  const TermType type;  ///< element of {sumOfSqr, inequality, equality}
  mlr::String name;
  bool active;
  arr target, prec;     ///< optional linear, potentially time-dependent, rescaling (with semantics of target & precision)

  Task(TaskMap* m, const TermType& type) : map(*m), type(type), active(true){}

  void setCostSpecs(int fromTime, uint toTime,
                    const arr& _target=ARR(0.),
                    double _prec=1.);
  bool isActive(uint t){ return (active && prec.N>t && prec(t)); }

  static Task* newTask(const Node* specs, const ors::KinematicWorld& world, uint Tinterval, uint Tzero=0); ///< create a new Task from specs
};



//===========================================================================
//
/// This class allows you to DESCRIBE a motion planning problem, nothing more
//

struct MotionProblem : KOrderMarkovFunction{
  ors::KinematicWorld& world;  ///< the original world, which also defines the 'start conditions'
  WorldL configurations;       ///< copies for each time slice; including kinematic switches; only these are optimized
  bool useSwift;
  
  /// task cost descriptions
  mlr::Array<Task*> tasks;

  /// kinematic switches along the motion
  mlr::Array<ors::KinematicSwitch*> switches;

  //-- trajectory length and tau
  uint T;       ///< number of time steps
  double tau;   ///< duration of single step
  uint k_order; ///< determine the order of the KOMO problem (default 2)
  
  //-- return values of an optimizer
  arrA phiMatrix;                  ///< storage of all features in all time slices
  mlr::Array<TermTypeA> ttMatrix;  ///< storage of all feature-types in all time slices
  arr dualSolution;                ///< the dual solution computed during constrained optimization

  struct OpenGL *gl; //internal only: used in 'displayTrajectory'

  MotionProblem(ors::KinematicWorld& originalWorld, bool useSwift=true);
  ~MotionProblem();
  
  MotionProblem& operator=(const MotionProblem& other);

  /// setting the numer of time steps and total duration in seconds
  void setTiming(uint steps, double duration);

  //-- setting costs in a task space
  void parseTasks(const Graph& specs, int Tinterval=-1, uint Tzero=0);     ///< read all tasks from a graph
  bool parseTask(const Node *n, int Tinterval=-1, uint Tzero=0);           ///< read a single task from a node-spec
  Task* addTask(const char* name, TaskMap *map, const TermType& termType); ///< manually add a task

  //-- initialization
  void setupConfigurations();   ///< this creates the @configurations@, that is, copies the original world T times (after setTiming!)
  arr getInitialization();      ///< this reads out the initial state trajectory after 'setupConfigurations'

  //-- methods accessed by the optimizers
  void set_x(const arr& x);            ///< set the state trajectory of all configurations
  void phi_t(arr& phi, arr& J, TermTypeA& tt, uint t); ///< read out the general task vector and its Jacobian for time slice t (this is APPENDING to phi and J)
  uint dim_phi(uint t);
  uint dim_g(uint t);
  uint dim_h(uint t);
  uint get_T() { return T; }
  uint get_k() { return k_order; }
  uint dim_x(uint t) { return configurations(t+k_order)->getJointStateDimension(); }

  //-- info on the costs
  StringA getPhiNames(uint t);
  void reportFull(bool brief=false, ostream& os=std::cout);
  void costReport(bool gnuplt=true); ///< also computes the costMatrix
  Graph getReport();

  //-- helpers
  void temporallyAlignKinematicSwitchesInConfiguration(uint t);
  void displayTrajectory(int steps, const char *tag, double delay=0.);

  /// inverse kinematics problem (which is the special case T=0) returned as a @ConstrainedProblem@
  /// as input to optimizers
  ConstrainedProblem InvKinProblem(){
    return [this](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void {
      this->inverseKinematics(phi, J, H, tt, x);
    };
  }
  void inverseKinematics(arr& y, arr& J, arr& H, TermTypeA& tt, const arr& x);
};


//===========================================================================
//
// basic helpers
//

arr getH_rate_diag(ors::KinematicWorld& world);
void sineProfile(arr& q, const arr& q0, const arr& qT,uint T);
arr reverseTrajectory(const arr& q);
void getVel(arr& v, const arr& q, double tau);
void getAcc(arr& a, const arr& q, double tau);

