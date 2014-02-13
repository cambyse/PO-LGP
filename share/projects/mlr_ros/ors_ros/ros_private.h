#ifndef ROS_PRIVATE_H
#define ROS_PRIVATE_H

#include <Ors/ors.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class TF_Sender {
private:
  tf::TransformBroadcaster tf_sender;

public:
  void publish_bodies(const ros::Time& time, const ors::KinematicWorld& w);
};

class MarkerSender {
private:
  ros::NodeHandle node_handle;
  ros::Publisher marker_pub;
  std::map<int,bool> known_markers;

public:
  MarkerSender();
  virtual ~MarkerSender();
  void publish_bodies(const ros::Time& time, const ors::KinematicWorld& w);
};

#endif // ROS_PRIVATE_H
