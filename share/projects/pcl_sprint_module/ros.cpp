#include <ros/ros.h>
#include <Core/array-vector.h>
#include <ros_msg/JointState.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pr2/roscom.h>
#include <tf/transform_datatypes.h>


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
      msg.markers[i].pose.position.x = list_primitives[i].first->values[0];
      msg.markers[i].pose.position.y = list_primitives[i].first->values[1];
      msg.markers[i].pose.position.z = list_primitives[i].first->values[2];



      msg.markers[i].header.frame_id = "/head_mount_kinect_rgb_optical_frame";

      msg.markers[i].ns = "ns";
      msg.markers[i].id = i;



      //Sphere
    if(list_primitives[i].second == 0){
        msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        msg.markers[i].scale.x = 2*list_primitives[i].first->values[3];
        msg.markers[i].scale.y = 2*list_primitives[i].first->values[3];
        msg.markers[i].scale.z = 2*list_primitives[i].first->values[3];

        msg.markers[i].color.a = 1.0;
        msg.markers[i].color.r = 1.0;
    }
    //Cylinder
    if(list_primitives[i].second == 1){
        msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
        tf::Vector3 axis_vector(list_primitives[i].first->values[3], list_primitives[i].first->values[4],
                list_primitives[i].first->values[5]);

        double height = sqrt(list_primitives[i].first->values[3]*list_primitives[i].first->values[3] + list_primitives[i].first->values[4]*list_primitives[i].first->values[4]+ list_primitives[i].first->values[5]*list_primitives[i].first->values[5]);

        tf::Vector3 up_vector(0.0, 0.0, 1.0);
        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
        q.normalize();
        geometry_msgs::Quaternion cylinder_orientation;
        tf::quaternionTFToMsg(q, cylinder_orientation);

        //orientation

        msg.markers[i].pose.orientation = cylinder_orientation;

        msg.markers[i].scale.x = 2*list_primitives[i].first->values[6];
        msg.markers[i].scale.y = 2*list_primitives[i].first->values[6];
        msg.markers[i].scale.z = height;//2*list_primitives[i].first->values[6];

        msg.markers[i].color.a = 1.0;
        msg.markers[i].color.g = 1.0;

    }

  }
  s->pub.publish(msg);
  ros::spinOnce();
  cout <<"*** I'm publishing!" <<endl;
}
