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


#include "AlvarFilter.h"
#include <unordered_set>

AlvarFilter::AlvarFilter():
    Module("AlvarFilter", 0){}


void AlvarFilter::open(){
  ros::init(mlr::argc, mlr::argv, "alvar_filter", ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle;
  std::cout << "Opening Alvar filter. " << std::endl;
}

void AlvarFilter::close(){
  nh->shutdown();
  delete nh;
}

void AlvarFilter::step(){
  std::cout << "Step." << std::endl;

  ar_pose_markers.waitForNextRevision();

  raw_markers = ar_pose_markers.get();
  old_markers = ar_pose_markers_tracked.get();

  if ((old_markers.size() == 0) && (raw_markers.size() == 0))
      return;

  createCostMatrix(costs);

  ha = new Hungarian(costs);
  ha->minimize();

  //std::cout << "I have: " << ha->starred.dim(0) << " matches." << std::endl;

  std::vector<Cluster> new_tracks;

  std::unordered_set<int> matched_ids;

  uint num_old = old_clusters.size();
  uint num_new = raw_clusters.size();


//  std::cout << num_old << ' ' << num_new << std::endl;

  for (uint i = 0; i < ha->starred.dim(0); ++i )
  {
    uint col = ha->starred[i]().maxIndex();
    // 3 cases:
    // 1) Existed before, no longer exists. If i > num_new
    // 2) Didn't exist before. This happens iff col >= num_old
    // 3) Existed before and still exists.

    // Existed before, doesn't exist now.
    if ( i >= num_new )
    {
      old_clusters.at(col).relevance *= relevance_decay_factor;
      new_tracks.push_back(old_clusters.at(col));
    }
    else
    {
      if ( ( col < num_old ) && (costs(i, col) < distance_threshold) )// Existed before
      {
        //std::cout<< "Existed before." << std::endl;
        raw_clusters.at(i).id = old_clusters.at(col).id;
        new_tracks.push_back( raw_clusters.at(i) );
      }
      else // This didn't exist before. Add it in
      {
        //std::cout<< "Didn't exist before, or not close enough." << costs(i,col) << std::endl;
        raw_clusters.at(i).id = maxId;
        maxId++;
        new_tracks.push_back(raw_clusters.at(i));
        //std::cout << "Didn't exist before. Col >= num_old: " << col << ' ' << num_old << std::endl;
      }
    }
    matched_ids.insert(new_tracks.at(i).id);
    //std::cout << "Assigning new\t" << i << "\tMatches:\t" << new_tracks.at(i).id << "\t Relevance: " << new_tracks.at(i).relevance << std::endl;
  }
  for (uint i = 0; i < num_old; ++i)
  {
    //std::cout << "Seeing if: " << old_clusters.at(i).id << " exists." << std::endl;
    if ( matched_ids.find(old_clusters.at(i).id) == matched_ids.end() )
    {
      old_clusters.at(i).relevance *= relevance_decay_factor;
      new_tracks.push_back(old_clusters.at(i));
      //std::cout << "Assigning old\t" << old_clusters.at(i).id << "\t Relevance: " << new_tracks.at(new_tracks.size() - 1).relevance << std::endl;
    }
  }

  std::vector<Cluster> to_assign;
  for (uint i = 0; i < new_tracks.size(); ++i)
  {
    if(new_tracks.at(i).relevance<relevance_threshold)
    {
      continue;
    }
    else
    {
      //std::cout << new_tracks.at(i).id << ' ';
      to_assign.push_back(new_tracks.at(i));
    }
  }
  //std::cout << std::endl;

  visualization_msgs::MarkerArray new_markers;
  for (uint i = 0; i < to_assign.size(); ++i)
  {
    new_markers.markers.push_back(conv_cluster2Marker(to_assign.at(i)));
  }
  pub.publish(new_markers);

  //std::cout << "Assigned: " << to_assign.size() << std::endl;
  tracked_clusters.set() = to_assign;
  //mlr::wait();
  //std::cout << "Assigned. " << std::endl;
  delete ha;
}

void AlvarFilter::convertMessage(const AlvarMarkers& markers) {
  // go through currently provided markers
  raw_markers.clear();

  for(auto & marker : markers){
    mlrAlvar new_marker;
    new_marker.id = marker.id;
    new_marker.ros_alvar = marker;
    new_marker.pos = conv_
    raw_markers.push_back(new_marker);
  }
}


void AlvarFilter::createCostMatrix(arr& costs){

  // 3 cases:
  int num_old = old_markers.size();
  int num_new = raw_markers.size();
  int dims = std::max(num_old, num_new);

  if (dims == 0)
    return;

  costs = ones(dims, dims) * -1.0;
  for (int i = 0; i < num_new; i++)
  {
    for (int j = 0; j < num_old; j++)
    {
      costs(i,j) = length(raw_clusters[i].mean - raw_markers[j].mean);
      //std::cout << "Clusters: " << i << ' ' << j << "   dist: " << costs(i,j) << std::endl;
    }
  }
  double max_costs = costs.max();
  for (int i = 0; i < dims; i++)
  {
    for (int j = 0; j < dims; j++)
    {
      if (( i >= num_new ) || ( j >= num_old ))
        costs(i,j) = max_costs;
    }
  }
  //std::cout << "Cost created." << std::endl;
}



REGISTER_MODULE(AlvarFilter)

