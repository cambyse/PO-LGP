#include "ors_locker.h"
#include "ors.h"

namespace ors {
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
