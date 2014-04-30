#ifndef _ORS_LOCKER_H_
#define _ORS_LOCKER_H_

#include "ors.h"

namespace ors {
struct JointLocker {
  virtual bool lock() = 0;   ///< returns true if in _this_ timestep the joint 
                             ///  should be locked
  virtual bool unlock() = 0; ///< returns true if in _this_ timestep the joint 
                             ///  should be unlocked
  virtual bool locked() = 0; ///< returns true if the joint is currently locked. 
                             ///  this is true after lock() returned true and false 
                             ///  after unlock() returned true
};

struct JointDependentLocker : public JointLocker {
  bool joint_locked;
  Joint* dependent_joint;
  double lower_locked_limit;
  double upper_locked_limit;

  JointDependentLocker();

  virtual bool lock();
  virtual bool unlock();
  virtual bool locked();
};

}
#endif
