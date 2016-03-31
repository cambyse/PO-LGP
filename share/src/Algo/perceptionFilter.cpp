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

#include <unordered_set>
#include <Algo/perceptionFilter.h>
#include <geometry_msgs/PoseArray.h>

#ifdef MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  namespace ar = ar_track_alvar;
#else // Assuming INDIGO or later
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  namespace ar = ar_track_alvar_msgs;
#endif


Filter::Filter():
    Module("Filter", 0){}

void Filter::open(){
  ros::init(mlr::argc, mlr::argv, "cluster_filter", ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle;
  tabletop_pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);
  //alvar_pub = nh->advertise<ar::AlvarMarkers>("/tracked_ar_pose_marker", 1);
  alvar_pub = nh->advertise<geometry_msgs::PoseArray>("/tracked_ar_pose_marker", 1);
}

void Filter::close()
{
  nh->shutdown();
  delete nh;
}

visualization_msgs::Marker conv_FilterObject2Marker(const FilterObject& object)
{
  visualization_msgs::Marker new_marker;
  new_marker.type = visualization_msgs::Marker::POINTS;
  new_marker.points = conv_arr2points(object.Cluster::points);
  new_marker.id = object.id;
  new_marker.scale.x = .001;
  new_marker.scale.y = .001;
  new_marker.lifetime = ros::Duration(0.5);
  new_marker.header.stamp = ros::Time(0.);
  new_marker.header.frame_id = object.Cluster::frame_id;

  new_marker.color.a = object.relevance;
  new_marker.color.r = (double)((new_marker.id*10000)%97)/97;
  new_marker.color.g = (double)((new_marker.id*10000)%91)/91;
  new_marker.color.b = (double)((new_marker.id*10000)%89)/89;

  return new_marker;
}

ar::AlvarMarker conv_FilterObject2Alvar(const FilterObject& object)
{
  ar::AlvarMarker new_marker;
  new_marker.header.frame_id = object.Alvar::frame_id;
  new_marker.pose.pose.position.x = object.Alvar::position(0);
  new_marker.pose.pose.position.y = object.Alvar::position(1);
  new_marker.pose.pose.position.z = object.Alvar::position(2);
  new_marker.pose.pose.orientation.x = object.Alvar::quaternion.x;
  new_marker.pose.pose.orientation.y = object.Alvar::quaternion.y;
  new_marker.pose.pose.orientation.z = object.Alvar::quaternion.z;
  new_marker.pose.pose.orientation.w = object.Alvar::quaternion.w;
  new_marker.id = object.id;
  return new_marker;
}

geometry_msgs::Pose conv_FilterObject2AlvarVis(const FilterObject& object)
{
  geometry_msgs::Pose new_marker;
  new_marker.position.x = object.Alvar::position(0);
  new_marker.position.y = object.Alvar::position(1);
  new_marker.position.z = object.Alvar::position(2);
  new_marker.orientation.x = object.Alvar::quaternion.x;
  new_marker.orientation.y = object.Alvar::quaternion.y;
  new_marker.orientation.z = object.Alvar::quaternion.z;
  new_marker.orientation.w = object.Alvar::quaternion.w;
  return new_marker;
}


void Filter::step()
{
  std::cout << "Filter step" << std::endl;

  perceptual_inputs.waitForNextRevision();

  FilterObjects perceptualInputs = perceptual_inputs.get();
  FilterObjects objectDatabase = object_database.get();


  // If empty inputs, do nothing.
  if (perceptualInputs.N == 0)
    return;


  FilterObjects filteredInputs;
  FilterObjects matchedSubsetFromDatabase, matchedSubsetFromPerceptualInputs;

  arr matched_objects = zeros(objectDatabase.N);

  // For each type of inputs, run the algorithm.
  for (auto const& type : {FilterObject::FilterObjectType::alvar, FilterObject::FilterObjectType::cluster, FilterObject::FilterObjectType::plane})
  {
    matchedSubsetFromDatabase.clear();
    matchedSubsetFromPerceptualInputs.clear();

    // Grab the subset from the inputs matching this type
    matchedSubsetFromPerceptualInputs.clear();
    for (uint i = 0; i < perceptualInputs.N; i++)
    {
      if (perceptualInputs(i).type == type)
      {
        matchedSubsetFromPerceptualInputs.append(perceptualInputs(i));
      }
    }

    // Find all matching object of this type
    for (uint i = 0; i < objectDatabase.N; i++)
    {
      if (matched_objects(i) == 1)
        continue;

      if (objectDatabase(i).type == type)
      {
        matchedSubsetFromDatabase.append(objectDatabase(i));
        matched_objects(i) = 1;
      }
    }

    if (matchedSubsetFromPerceptualInputs.N == 0 && matchedSubsetFromDatabase.N == 0)
      continue;

    // Create bipartite costs
    costs = createCostMatrix(matchedSubsetFromPerceptualInputs, matchedSubsetFromDatabase);

    // Run Hungarian algorithm
    ha = new Hungarian(arr(costs));
    ha->minimize();

    // Now we have the optimal matching. Assign values.
    FilterObjects assignedObjects = assign(matchedSubsetFromPerceptualInputs, matchedSubsetFromDatabase);

    for (uint i = 0; i < assignedObjects.N; i++)
      filteredInputs.append(assignedObjects(i));

    delete ha;
  }

  // Add in any unmatched objects
  for (uint i = 0; i < objectDatabase.N; i++)
  {
    if (matched_objects(i) == 0)
    {
      filteredInputs.append(objectDatabase(i));
    }
  }

  // Set the access.
  object_database.set() = filteredInputs;


  visualization_msgs::MarkerArray cluster_markers;
  //ar::AlvarMarkers ar_markers;
  geometry_msgs::PoseArray ar_markers;

  int alvar_count = 0, cluster_count = 0;

  for (uint i = 0; i < filteredInputs.N; i++)
  {
    switch ( filteredInputs(i).type )
    {
      case FilterObject::FilterObjectType::alvar:
        alvar_count++;
        ar_markers.poses.push_back(conv_FilterObject2AlvarVis(filteredInputs(i)));
        ar_markers.header.frame_id = filteredInputs(i).Alvar::frame_id;
        //ar_markers.markers.push_back(conv_FilterObject2AlvarVis(filteredInputs(i)));
        break;
      case FilterObject::FilterObjectType::cluster:
        cluster_count++;
        cluster_markers.markers.push_back(conv_FilterObject2Marker(filteredInputs(i)));
        break;
      default:
        break;
    }
  }
  if (cluster_markers.markers.size() > 0)
    tabletop_pub.publish(cluster_markers);

  //if (ar_markers.markers.size() > 0)
  if (ar_markers.poses.size() > 0)
    alvar_pub.publish(ar_markers);

  std::cout << "Set the database: " << filteredInputs.N << ' ' << alvar_count << ' ' << cluster_count << std::endl;

}

FilterObjects Filter::assign(const FilterObjects& perceps, const FilterObjects& database)
{
  FilterObjects new_objects;
  std::unordered_set<int> matched_ids;

  uint num_old = database.N;
  uint num_new = perceps.N;

  for (uint i = 0; i < ha->starred.dim(0); ++i )
  {
    uint col = ha->starred[i]().maxIndex();
    // 3 cases:
    // 1) Existed before, no longer exists. If i > num_new
    // 2) Existed before and still exists. If costs < distannce_threshold
    // 3) Didn't exist before. This happens iff col >= num_old

    // Existed before, doesn't exist now.
    if ( i >= num_new )
    {
      //std::cout<< "Existed before, doesn't now." << std::endl;
      FilterObject new_obj = database(col);
      new_obj.relevance *= relevance_decay_factor;
      new_objects.append(new_obj);
    }
    else
    {
      if ( ( col < num_old ) && (costs(i, col) < distance_threshold) )// Existed before
      {
        //std::cout<< "Existed before, does now" << std::endl;
        FilterObject new_obj = perceps(i);
        new_obj.id = database(col).id;
        new_objects.append( new_obj );
      }
      else // This didn't exist before. Add it in
      {
        //std::cout<< "Didn't exist before, or not close enough." << std::endl;
        FilterObject new_obj = perceps(i);
        if (new_obj.type != FilterObject::FilterObjectType::alvar)
        {
          new_obj.id = maxId;
          maxId++;
        }
        new_objects.append(new_obj);
        //std::cout << "Didn't exist before. Col >= num_old: " << col << ' ' << num_old << std::endl;
      }
    }
    matched_ids.insert(new_objects(i).id);
    //std::cout << "Assigning \t" << i << "\tMatches:\t" << matched_ids(i).id << "\t Relevance: " << perceps(i).relevance << std::endl;
  }

  // For each of the old objects, update the relevance factor.
  for ( uint i = 0; i < num_old; ++i )
  {
    //std::cout << "Seeing if: " << old_clusters.at(i).id << " exists." << std::endl;
    if ( matched_ids.find(database(i).id) == matched_ids.end() )
    {
      FilterObject new_obj = database(i);
      new_obj.relevance *= relevance_decay_factor;
      new_objects.append(new_obj);
      //std::cout << "Assigning old\t" << old_clusters.at(i).id << "\t Relevance: " << new_tracks.at(new_tracks.size() - 1).relevance << std::endl;
    }
  }

  FilterObjects cleaned;
  uint count = new_objects.N;
  for ( uint i = 0; i < count; ++i )
  {
    if(new_objects(i).relevance > relevance_threshold)
    {
      cleaned.append(new_objects(i));
    }
  }

  return cleaned;
}

arr Filter::createCostMatrix(const FilterObjects& newObjects, const FilterObjects& oldObjects)
{
  // First, make a padded out matrix to ensure it is bipartite.
  uint num_new = newObjects.N;
  uint num_old = oldObjects.N;
  uint dims = std::max(num_old, num_new);
  arr costs = ones(dims, dims) * -1.0;

  // Assign costs
  for (uint i = 0; i < num_new; ++i)
  {
    for (uint j = 0; j < num_old; ++j)
    {
      costs(i,j) = newObjects(i).idMatchingCost(oldObjects(j));
    }
  }

  // For every element that hasn't been set, set the costs to the max.
  double max_costs = costs.max();

  for (uint i = 0; i < dims; ++i)
  {
    for (uint j = 0; j < dims; ++j)
    {
      if (( i >= num_new ) || ( j >= num_old ))
        costs(i,j) = max_costs;
    }
  }
  return costs;
}
