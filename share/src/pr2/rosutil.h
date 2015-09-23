#pragma once

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

void rosCheckInit(const char* module_name="pr2_module");
bool rosOk();

//-- convert pose to other things
ors::Transformation cvrt_pose2transformation(const tf::Transform&);
ors::Transformation cvrt_pose2transformation(const geometry_msgs::Pose&);
void cvrt_pose2transXYPhi(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose);

double cvrt_time2double(const ros::Time& time);
timespec cvrt_time2timespec(const ros::Time&);




ors::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener);

arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts);
arr conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts);
std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts);


