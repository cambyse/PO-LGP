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

#include "motion.h"
#include "taskMaps.h"

/**
 * @file
 * With the feedback control we can define motions for operation space control.
 *
 * We simply define a set of motions via CtrlTasks/ConstraintForceTask and run
 * them.
 */


//===========================================================================
/**
 * A CtrlTask defines a motion in operational space.
 */
struct CtrlTask{ //TODO: rename/refactor to become LinearAccelerationLaw (LAW) in task spaces
  TaskMap& map;
  MT::String name;
  bool active;
  double prec;

  /// @{ @name Parameters that define the linear acceleration control law
  arr y_ref; ///< position reference
  arr v_ref; ///< velocity reference
  double Pgain; ///< proportional gain
  double Dgain; ///< derivative gain
  double maxVel, maxAcc;
  /// @}

  /// @{ @name Parameters that define the integral force feedback control law
  arr f_ref;
  double f_Igain;

  /// Option for metric (difference) in task space: flip sign if scalar product is negative (for quaternion targets)
  bool flipTargetSignOnNegScalarProduct;

  /// @{ @name The actual state when LAST getDesiredAcceleration was called
  arr y, v;
  /// @}

  CtrlTask(TaskMap* map) : map(*map), active(true), prec(0.), Pgain(0.), Dgain(0.), maxVel(0.), maxAcc(0.), f_Igain(0.), flipTargetSignOnNegScalarProduct(false){}
  CtrlTask(const char* name, TaskMap* map, double decayTime, double dampingRatio, double maxVel, double maxAcc);
  CtrlTask(const char* name, TaskMap& map, Graph& params);

  void setTarget(const arr& yref, const arr& vref=NoArr);
  void setGains(double Pgain, double Dgain);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error

  arr getDesiredAcceleration(const arr& y, const arr& ydot);

  void getForceControlCoeffs(arr& f_des, arr& u_bias, arr& KfL, arr& J_ft, const ors::KinematicWorld& world);

  void reportState(ostream& os);
};

//===========================================================================

struct ConstraintForceTask{
  TaskMap& map;
  MT::String name;
  bool active;

  double desiredForce;
  CtrlTask desiredApproach;

  ConstraintForceTask(TaskMap* m):map(*m), active(true), desiredForce(0.), desiredApproach(m){}

  void updateConstraintControl(const arr& g, const double& lambda_desired);
};

//===========================================================================

/**
 * FeedbackMotionControl contains all individual motions/CtrlTasks.
 */
struct FeedbackMotionControl : MotionProblem {
  MT::Array<CtrlTask*> tasks;
  MT::Array<ConstraintForceTask*> forceTasks;
  CtrlTask qitselfPD;
  arr H_rate_diag;

  FeedbackMotionControl(ors::KinematicWorld& _world, bool useSwift=true);

  /// @{ @name adding tasks
  CtrlTask* addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map);
  CtrlTask* addPDTask(const char* name,
                    double decayTime, double dampingRatio,
                    DefaultTaskMapType type,
                    const char* iShapeName=NULL, const ors::Vector& ivec=NoVector,
                    const char* jShapeName=NULL, const ors::Vector& jvec=NoVector);
  ConstraintForceTask* addConstraintForceTask(const char* name, TaskMap *map);
  /// @}

  void getCostCoeffs(arr& c, arr& J); ///< the general (`big') task vector and its Jacobian
  arr getDesiredConstraintForces(); ///< J^T lambda^*
  arr operationalSpaceControl();
  void updateConstraintControllers();
  void reportCurrentState();
};

//===========================================================================
