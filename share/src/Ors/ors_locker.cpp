#include "ors_locker.h"
#include "ors.h"
#include <devTools/logging.h>

SET_LOG(locker, DEBUG);

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
      DEBUG(locker, "lock");
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
      DEBUG(locker, "unlock");
      joint_locked = false;
      return true;  
    }
    else return false;
  }
  bool JointDependentLocker::locked() {
    return joint_locked;  
  }
};
