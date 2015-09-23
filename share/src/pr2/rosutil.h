#pragma once

#include <tf/transform_listener.h>
#include <std_msgs/ColorRGBA.h>

//===========================================================================
#include <Core/module.h>
#include <Core/array.h>
#include <Core/geo.h>
#include <Ors/ors.h>


//===========================================================================
//
// utils
//

namespace tf{ class Transform; }

bool rosOk();
//void rosCheckInit(const char* module_name="pr2_module");
ors::Transformation ros_cvrt(const tf::Transform&);
timespec ros_cvrt(const ros::Time&);


ors::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener);

arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts);
arr conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts);
std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts);

/*
//templace <class >class ROSSUB_##var_name : public Module { \
//public: \
//  ACCESS(msg_type, var_name) \
//  ROSSUB_##var_name() : Module(#var_name) {} \
//  void open() { \
//    rosCheckInit(); \
//    this->_nh = new ros::NodeHandle; \
//    this->_sub  = this->_nh->subscribe( \
//      topic_name, 1, &ROSSUB_##var_name::callback, this); \
//  } \
//  void step() {} \
//  void close() { \
//    this->_nh->shutdown(); \
//    delete _nh; \
//  } \
//  void callback(const msg_type::ConstPtr& msg) { \
//    this->var_name.set() = *msg; \
//  } \
//private: \
//  ros::NodeHandle* _nh; \
//  ros::Subscriber _sub; \
//  \
//};
*/
