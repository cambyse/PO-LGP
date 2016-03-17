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

//#include "activity.h"
#include <Core/thread.h>
#include <Core/module.h>
#include <Core/array.h>
#include <pr2/roscom.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>


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
visualization_msgs::Marker conv_cluster2Marker(const Cluster& cluster);
Cluster conv_Marker2Cluster(const visualization_msgs::Marker& marker);

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

  int last_revision;

  ClusterFilter();
 // virtual ~ClusterFilter();

  virtual void open();
  virtual void step();
  virtual void close();

  // Main filtering function
  void convertMessage(const visualization_msgs::MarkerArray& msg);
  void createCostMatrix(arr& costs);
/*  void HungarianAlgorithm(arr& costs);

  bool checkIfFinished(arr& costs);
  bool recurse(arr& costs, const uint row, const bool verbose=false);

  void markRows(const arr& costs, const arr& assigned, arr& marked_rows, arr& marked_cols, const int col );
  void markCols(const arr& costs, const arr& assigned, arr& marked_rows, arr& marked_cols, const int row );
*/
};
