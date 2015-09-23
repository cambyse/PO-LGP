#include "rosutil.h"

ors::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener){
  ors::Transformation X;
  try{
    tf::StampedTransform transform;
    listener.lookupTransform(from, to, ros::Time(0), transform);
    X = ros_cvrt(transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return X;
}


arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts){
  uint n=pts.size();
  arr P(n,3);
  for(uint i=0;i<n;i++){
    P(i,0) = pts[i].x;
    P(i,1) = pts[i].y;
    P(i,2) = pts[i].z;
  }
  return P;
}

arr conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts){
  uint n=pts.size();
  arr P(n,3);
  for(uint i=0;i<n;i++){
    P(i,0) = pts[i].r;
    P(i,1) = pts[i].g;
    P(i,2) = pts[i].b;
  }
  return P;
}


std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts){
  uint n=pts.d0;
  std::vector<geometry_msgs::Point> P(n);
  for(uint i=0;i<n;i++){
    P[i].x = pts(i, 0);
    P[i].y = pts(i, 1);
    P[i].z = pts(i, 2);
  }
  return P;
}
