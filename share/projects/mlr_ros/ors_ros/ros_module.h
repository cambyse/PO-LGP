#ifndef ROS_MODULE_H
#define ROS_MODULE_H

#include <Core/thread.h>
#include <Ors/ors.h>

BEGIN_MODULE(RosTf)
  ACCESS(mlr::KinematicWorld, world)
  void publish(const mlr::KinematicWorld&);
END_MODULE()

BEGIN_MODULE(PhysicsMenu)
  ACCESS(bool, do_physics)
END_MODULE()

#endif // ROS_MODULE_H
