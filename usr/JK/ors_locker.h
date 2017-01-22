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


#ifndef _ORS_LOCKER_H_
#define _ORS_LOCKER_H_

#include "kin.h"

namespace mlr {
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
