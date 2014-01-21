#include "ros_module.h"
#include <Core/geo.h>

#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace {
  tf::Vector3 convert(const ors::Vector& v) {
    return tf::Vector3(v.x, v.y, v.z);
  }
  tf::Quaternion convert(const ors::Quaternion& v) {
    return tf::Quaternion(v.x, v.y, v.z, v.w);
  }
  tf::Transform convert(const ors::Transformation& t) {
    return tf::Transform(convert(t.rot), convert(t.pos));
  }
}

void RosTf::open() {
    tf_sender = new tf::TransformBroadcaster();
}
void RosTf::close() {
  delete tf_sender;
  tf_sender = NULL;
}

void RosTf::step() {
  Access_typed<ors::KinematicWorld>::ReadToken w(world.get());
  ros::Time timestamp(ros::Time::now());
  std::ostringstream name;

  for(ors::Joint* j : ((ors::KinematicWorld&) w).joints) {
      name.str("");
      if(((const char*)j->name) != NULL) {
          name << (const char*)j->name;
      } else {
          name << "joint-" << j->index;
      }
      tf_sender->sendTransform(tf::StampedTransform(convert(j->X), timestamp, "world",
                                                    name.str()));
  }
}
