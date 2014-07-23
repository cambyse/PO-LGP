#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <pcl/ModelCoefficients.h>

struct Ros_publishPrimitives{
  struct sRos_publishPrimitives *s;
  Ros_publishPrimitives();
  ~Ros_publishPrimitives();
  void publish(std::vector<std::pair<pcl::ModelCoefficients::Ptr,int>>& list_primitives);
};
