#ifndef ROS_MODULE_H
#define ROS_MODULE_H

#include <Core/module.h>
#include <Ors/ors.h>
#include <tf/transform_broadcaster.h>

BEGIN_MODULE(RosTf) ACCESS(ors::KinematicWorld, world) tf::TransformBroadcaster *tf_sender; END_MODULE()

#endif // ROS_MODULE_H
