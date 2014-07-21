#include "ros_module.h"
#include <Core/geo.h>

#include <sstream>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Pose.h>
#include "ros_private.h"

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

void TF_Sender::publish_bodies(const ros::Time& timestamp, const ors::KinematicWorld& w) {
  std::ostringstream name;

  for(ors::Body* b : w.bodies) {
      name.str("");
      name << "body-" << b->index;
      tf_sender.sendTransform(tf::StampedTransform(convert(b->X), timestamp, "world",
                                                    name.str()));
  }
}


struct sRosTf {
  TF_Sender tf;
  MarkerSender markers;
};

namespace {
  visualization_msgs::Marker::_pose_type& convert(visualization_msgs::Marker::_pose_type& left, const ors::Transformation &t) {
    left.position.x = t.pos.x;
    left.position.y = t.pos.y;
    left.position.z = t.pos.z;
    left.orientation.x = t.rot.x;
    left.orientation.y = t.rot.y;
    left.orientation.z = t.rot.z;
    left.orientation.w = t.rot.w;
    return left;
  }
}

MarkerSender::MarkerSender() {
  marker_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
}
MarkerSender::~MarkerSender() {
  node_handle.shutdown();
}

void MarkerSender::publish_bodies(const ros::Time& timestamp, const ors::KinematicWorld& w) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp = timestamp;
  marker.ns = "ors";

  for(ors::Body* b : w.bodies) {
      marker.id = b->index;
      marker.type = visualization_msgs::Marker::SPHERE; //FIXME!
      marker.action = known_markers.find(marker.id) == known_markers.end() ?
            visualization_msgs::Marker::ADD : visualization_msgs::Marker::MODIFY;
      if(marker.action == visualization_msgs::Marker::ADD) {
          known_markers[marker.id] == true;
      }
      convert(marker.pose, b->X);
      marker.scale.x = .1;
      marker.scale.y = .1;
      marker.scale.z = .1;
      // green
      marker.color.a = 1;
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;

      marker_pub.publish(marker);
  }
}


void RosTf::open() {
  s = new sRosTf();
}
void RosTf::close() {
  delete s;
}

void RosTf::step() {
  publish(world.get());
}

void RosTf::publish(const ors::KinematicWorld& w) {
  ros::Time timestamp(ros::Time::now());
  s->tf.publish_bodies(timestamp, w);
  s->markers.publish_bodies(timestamp, w);
}

// PHYSICS MENU SERVER

using namespace visualization_msgs;
using namespace interactive_markers;

namespace  {
void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {

  }
}

struct sPhysicsMenu {
  InteractiveMarkerServer server;
  InteractiveMarker marker;
  InteractiveMarkerControl box_control, rotate_control;
  Marker box;

  sPhysicsMenu() : server("simple_marker") {
    marker.header.frame_id = "/world";
    marker.name = "test_marker";
    marker.description = "simple marker";


    box.type = Marker::CUBE;
    box.scale.x=.3;
    box.scale.y=.3;
    box.scale.z=.3;
    box.color.r=.5;
    box.color.g=.5;
    box.color.b=.5;
    box.color.a=1;

    box_control.always_visible = true;
    box_control.markers.push_back(box);
    box_control.interaction_mode = InteractiveMarkerControl::BUTTON; // get some feedback when selected
    marker.controls.push_back(box_control);

    rotate_control.name = "rotate_x";
    rotate_control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    marker.controls.push_back(rotate_control);

    server.insert(marker, boost::bind(&sPhysicsMenu::processFeedback, this, _1));
    server.applyChanges();
    cout << "interactive marker created" << endl;
  }

  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
                     << feedback->pose.position.x << ", " << feedback->pose.position.y
                     << ", " << feedback->pose.position.z
                     << ", etype: " << (int)feedback->event_type
                     << ", valid: " << (int)feedback->mouse_point_valid
                     );
  }
};

void PhysicsMenu::open() {
  s = new sPhysicsMenu();
}
void PhysicsMenu::close() {
  delete s;
}
void PhysicsMenu::step() {
  ros::spinOnce();
}

REGISTER_MODULE(RosTf)
REGISTER_MODULE(PhysicsMenu)
