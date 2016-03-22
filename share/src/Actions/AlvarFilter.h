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
#include <pr2/subscribeAlvarMarkers.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector>


struct AlvarFilter : Module{
  ACCESSname(AlvarMarkers, ar_pose_markers)

  ACCESSname(AlvarMarkers, ar_pose_markers_tracked)

  AlvarMarkers raw_markers;
  AlvarMarkers old_markers;

  // Convert them into our data
  std::vector<mlrAlvar> raw_markers_poses;
  std::vector<mlrAlvar> old_markers_poses;

  double relevance_decay_factor = 0.9;
  double relevance_threshold = 0.25;

  double distance_threshold = 0.5;

  ros::NodeHandle* nh;
  //ros::Subscriber sub;
  ros::Publisher pub;
  int maxId = 0;

  Hungarian *ha;
  arr costs;

  AlvarFilter();
 // virtual ~AlvarFilter();

  virtual void open();
  virtual void step();
  virtual void close();

  // Main filtering functions
  void convertMessage(const AlvarMarkers& markers);
  void createCostMatrix(arr& costs);


};
