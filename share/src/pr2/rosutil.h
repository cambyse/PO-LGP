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
ors::Transformation cvrt_pose2transformation(const tf::Transform&);
ors::Transformation cvrt_pose2transformation(const geometry_msgs::Pose&);
double cvrt2double(const ros::Time& time);

timespec cvrt_pose2transformation(const ros::Time&);


ors::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener);

arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts);
arr conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts);
std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts);


