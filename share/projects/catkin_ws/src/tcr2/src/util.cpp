#include "util.h"
#include <geometry_msgs/Point.h>

ors::Vector ros_to_ors_vector(geometry_msgs::Point& vec) {
  return ors::Vector(vec.x, vec.y, vec.z);  
}
