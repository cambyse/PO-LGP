#include <ros/ros.h>
#include <Core/array-vector.h>
#include <ros_msg/JointState.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pr2/roscom.h>


#include "ros.h"

struct sRos_publishPrimitives{
  Ros_publishPrimitives *base;
  ros::NodeHandle nh;
  ros::Publisher pub;
};

Ros_publishPrimitives::Ros_publishPrimitives():s(NULL){
  rosCheckInit();
  s = new sRos_publishPrimitives;
  s->base=this;
  s->pub = s->nh.advertise<visualization_msgs::MarkerArray>("/PclSprint/extractedPrimitives", 1);
}

Ros_publishPrimitives::~Ros_publishPrimitives(){
  s->nh.shutdown();
  delete s;
}

void Ros_publishPrimitives::publish(std::vector<std::pair<pcl::ModelCoefficients::Ptr,int>>& list_primitives){
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(list_primitives.size());
  for(uint i=0;i<list_primitives.size();i++){
    msg.markers[i].type = visualization_msgs::Marker::SPHERE;
    msg.markers[i].pose.position.x = list_primitives[i].first->values[0];
    msg.markers[i].pose.position.y = list_primitives[i].first->values[1];
    msg.markers[i].pose.position.z = list_primitives[i].first->values[2];
  }
  s->pub.publish(msg);
  ros::spinOnce();
  cout <<"*** I'm publishing!" <<endl;
}
