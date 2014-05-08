#ifndef _TCR2_UTIL_H_
#define _TCR2_UTIL_H_

#include <Ors/ors.h>
#include <geometry_msgs/Point.h>

ors::Vector ros_to_ors_vector(geometry_msgs::Point& vec);

#endif //_TCR2_UTIL_H_
