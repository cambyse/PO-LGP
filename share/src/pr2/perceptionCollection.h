#pragma once

#include <Algo/filterObject.h>
#include <visualization_msgs/MarkerArray.h>
#include <Core/module.h>

FilterObject conv_Marker2FilterObject(const visualization_msgs::Marker& marker);

struct Collector : Module{
  ACCESSname(visualization_msgs::MarkerArray, tabletop_clusters)
  ACCESSname(FilterObjects, perceptual_inputs)

//  ros::NodeHandle* nh;
//  ros::Publisher pub;

  Collector();

  virtual void open(){}
  virtual void step();
  virtual void close(){}

};
