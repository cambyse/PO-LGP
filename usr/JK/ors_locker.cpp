/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "ors_locker.h"
#include "ors.h"

namespace mlr {
  JointDependentLocker::JointDependentLocker() {
    joint_locked = 0; 
  }

  bool JointDependentLocker::lock() {
    if (joint_locked) { 
      return false; 
    }

    double dependent_angle = dependent_joint->Q.rot.getDeg();
    if (dependent_angle > lower_locked_limit &&
        dependent_angle < upper_locked_limit) {
      joint_locked = true;
      return true;  
    }
    else return false;
  }

  bool JointDependentLocker::unlock() {
    if (!joint_locked) {
      return false;
    }

    double dependent_angle = dependent_joint->Q.rot.getDeg();
    if (dependent_angle < lower_locked_limit ||
        dependent_angle > upper_locked_limit) {
      joint_locked = false;
      return true;  
    }
    else return false;
  }
  bool JointDependentLocker::locked() {
    return joint_locked;  
  }
};
