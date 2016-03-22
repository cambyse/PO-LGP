/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#pragma once

#include "Filtering.h"
#include <Core/thread.h>
#include <Core/module.h>
#include <Core/array.h>
#include <pr2/roscom.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

struct ClusterFilter : Module{
  ACCESSnew(std::vector<Cluster>, tracked_clusters)
  ACCESSnew(visualization_msgs::MarkerArray, tabletop_clusters)

  std::vector<Cluster> raw_clusters;
  std::vector<Cluster> old_clusters;

  double relevance_decay_factor = 0.9;
  double relevance_threshold = 0.25;

  double distance_threshold = 0.5;

  ros::NodeHandle* nh;
  //ros::Subscriber sub;
  ros::Publisher pub;
  int maxId = 0;

  Hungarian *ha;
  arr costs;

  ClusterFilter();
 // virtual ~ClusterFilter();

  virtual void open();
  virtual void step();
  virtual void close();

  // Main filtering functions
  void convertMessage(const visualization_msgs::MarkerArray& msg);
  void createCostMatrix(arr& costs);

};
