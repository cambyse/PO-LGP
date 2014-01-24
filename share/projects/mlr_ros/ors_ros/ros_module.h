#ifndef ROS_MODULE_H
#define ROS_MODULE_H

#include <Core/module.h>
#include <Ors/ors.h>
namespace tf {
  class TransformBroadcaster;
}

BEGIN_MODULE(RosTf)
ACCESS(ors::KinematicWorld, world)
tf::TransformBroadcaster *tf_sender;
void publish_bodies(const ors::KinematicWorld&);
END_MODULE()

#endif // ROS_MODULE_H
