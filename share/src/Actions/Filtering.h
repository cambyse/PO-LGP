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

// Our headers
#include <Core/array.h>
#include <pr2/roscom.h>

// ROS
#include <visualization_msgs/MarkerArray.h>

// General
#include <vector>

#ifdef MLR_ROS_INDIGO
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  using namespace ar_track_alvar_msgs;
#endif
#if MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  using namespace ar_track_alvar;
#endif

struct Cluster {
    Cluster() {}
    Cluster(int id,
            arr mean,
            arr points,
            double relevance,
            std::string frame_id):
        id(id),
        mean(mean),
        points(points),
        relevance(relevance),
        frame_id(frame_id)
    {}
    int id;
    arr mean;
    arr points;
    double relevance;
    std::string frame_id;
};

// Conversion functions.
visualization_msgs::Marker conv_cluster2Marker(const Cluster& cluster);
Cluster conv_Marker2Cluster(const visualization_msgs::Marker& marker);

struct mlrAlvar {
    mlrAlvar() {}
    mlrAlvar(int id,
            arr pos,
            AlvarMarker ros_alvar,
            double relevance,
            std::string frame_id):
        id(id),
        pos(pos),
        ros_alvar(ros_alvar),
        relevance(relevance),
        frame_id(frame_id)
    {}
    int id;
    arr pos;
    AlvarMarker ros_alvar;
    double relevance;
    std::string frame_id;
};

struct Hungarian {
  arr costs, starred, primed;
  uint dim;
  arr covered_rows;
  arr covered_cols;

  void minimize();
  void starZeros();
  void coverColumns();
  void prime();
  void makePath();
  void modifyCost();

  std::vector<uint> path_row;
  std::vector<uint> path_col;

  Hungarian(const arr& cost_matrix);
  ~Hungarian();
};
