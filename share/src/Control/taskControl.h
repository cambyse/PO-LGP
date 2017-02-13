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

#include <Motion/taskMaps.h>
#include <Algo/spline.h>

/**
 * @file
 * With the feedback control we can define motions for operation space control.
 *
 * We simply define a set of motions via CtrlTasks/ConstraintForceTask and run
 * them.
 */


struct CtrlTask;
typedef mlr::Array<CtrlTask*> CtrlTaskL;


//===========================================================================

/// a motion profile is a non-feedback(!) way to generate a task space reference path
/// [perhaps an adaptive phase, or Peter's adaptation to object motions, could be a modest way to incorporate feedback in the future]
struct MotionProfile{
  virtual ~MotionProfile(){}
  virtual void update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot) = 0;
  virtual bool isDone() = 0;
};

//===========================================================================

struct MotionProfile_Sine : MotionProfile{
  arr y_init, y_target;
  double t;
  double T;
  MotionProfile_Sine(const arr& y_target, double duration) : y_target(y_target), t(0.), T(duration){}
  virtual void update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual bool isDone(){ return t>=T; }
};

//===========================================================================

struct MotionProfile_PD: MotionProfile{
  arr y_ref, v_ref;
  arr y_target, v_target;
  double kp, kd;
  double maxVel, maxAcc;
  bool flipTargetSignOnNegScalarProduct;
  bool makeTargetModulo2PI;
  MotionProfile_PD();
  MotionProfile_PD(const arr& _y_target, double decayTime, double dampingRatio, double maxVel=0., double maxAcc=0.);
  MotionProfile_PD(const Graph& params);

  void setTarget(const arr& ytarget, const arr& vtarget=NoArr);
  void setGains(double _kp, double _kd);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error

  virtual void update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);

  arr getDesiredAcceleration();
  void getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y);

  double error();
  bool isConverged(double tolerance=1e-2);
  virtual bool isDone(){ return isConverged(); }
};

//===========================================================================

struct MotionProfile_Path: MotionProfile{
  mlr::Spline spline;
  double executionTime;
  double phase;
  MotionProfile_Path(const arr& path, double executionTime);
  virtual void update(arr& yRef, arr& ydotRef, double tau,const arr& y, const arr& ydot);
  virtual bool isDone(){ return phase>=1.; }
};

//===========================================================================

/** In the given task space, a task can represent: 1) a pos/vel ctrl task
 *  and/or 2) a compliance and/or 3) a force limit control */
struct CtrlTask{
  TaskMap *map;      ///< this defines the task space
  mlr::String name;  ///< just for easier reporting
  bool active;       ///< also non-active tasks are updates (states evaluated), but don't enter the TaskControlMethods

  //-- this is always kept up-to-date (in update)
  arr y, v, J_y;     ///< update() will evaluate these for a given kinematic configuration

  //-- pos/vel ctrl task
  MotionProfile *ref;  ///< non-NULL iff this is a pos/vel task
  arr y_ref, v_ref;    ///< update() will define compute these references (reference=NOW, target=FUTURE)
  arr prec;            ///< Cholesky(!) of C, not C itself: sumOfSqr(prec*(y-y_ref)) is the error, and prec*J the Jacobian

  //-- compliance task
  arr C;               ///< non-empty iff this is a compliance task; defines the task space compliance coefficients

  //-- if this is a force ctrl task
  arr f_ref;           ///< non-empty iff this is a force limit control task; defines the box limits (abs value in all dimensions)
  double f_alpha, f_gamma; ///< TODO

  CtrlTask(const char* name, TaskMap* map);
  CtrlTask(const char* name, TaskMap* map, double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask(const char* name, TaskMap* map, const Graph& params);
  ~CtrlTask();

  void update(double tau, const mlr::KinematicWorld& world);

  arr getPrec();
  void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const mlr::KinematicWorld& world);

  MotionProfile_PD& PD();

  void reportState(ostream& os);
};

//===========================================================================

void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& KfL, arr& J_ft, const mlr::KinematicWorld& world);
void fwdSimulateControlLaw(arr &Kp, arr &Kd, arr &u0, mlr::KinematicWorld& world);

//===========================================================================

/// implements a number of basic equations given a set of control tasks
struct TaskControlMethods {
  mlr::Array<CtrlTask*> tasks;
  arr Hmetric;           ///< defines the metric in q-space (or qddot-space)
  CtrlTask qNullCostRef; ///< defines the 'desired behavior' in qddot-space (regularization of operational space control)
  boolA lockJoints;

  TaskControlMethods(const mlr::KinematicWorld& world);

  CtrlTask* addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map);

  void updateCtrlTasks(double tau, const mlr::KinematicWorld& world);
  void lockJointGroup(const char *groupname, mlr::KinematicWorld& world, bool lockThem=true);

  arr inverseKinematics(arr& qdot);
  arr operationalSpaceControl();
  arr calcOptimalControlProjected(arr &Kp, arr &Kd, arr &u0, const arr& q, const arr& qdot, const arr& M, const arr& F); ///< returns the linearized control law
  arr getDesiredLinAccLaw(arr &Kp, arr &Kd, arr &u0, const arr& q, const arr& qdot); ///< returns the linearized control law
  arr getDesiredConstraintForces(); ///< J^T lambda^*
  void calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma, const mlr::KinematicWorld& world); ///< returns the force controller coefficients
  void reportCurrentState();
};

//===========================================================================
