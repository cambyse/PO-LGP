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


#include "ClusterFilterActivity.h"
#include <unordered_set>

ClusterFilter::ClusterFilter():
    Module("ClusterFilter", 0){}


//ClusterFilter::~ClusterFilter(){}
visualization_msgs::Marker conv_cluster2Marker(const Cluster& cluster)
{
  visualization_msgs::Marker new_marker;
  new_marker.type = visualization_msgs::Marker::POINTS;
  new_marker.points = conv_arr2points(cluster.points);
  new_marker.id = cluster.id;
  new_marker.scale.x = .001;
  new_marker.scale.y = .001;
  new_marker.lifetime = ros::Duration(0.5);
  new_marker.header.stamp = ros::Time(0.);
  new_marker.header.frame_id = cluster.frame_id;

  new_marker.color.a = cluster.relevance;
  new_marker.color.r = (double)((new_marker.id*10000)%97)/97;
  new_marker.color.g = (double)((new_marker.id*10000)%91)/91;
  new_marker.color.b = (double)((new_marker.id*10000)%89)/89;

  return new_marker;
}

Cluster conv_Marker2Cluster(const visualization_msgs::Marker& marker)
{
  arr pts = conv_points2arr(marker.points);
  arr mean = sum(pts,0)/(double)pts.d0;
  // Put it into our list

  Cluster new_cluster;
  new_cluster.mean = mean;
  //std::cout << "Mean: " << mean(0) << ' ' << mean(1) << ' ' << mean(2) << std::endl;

  new_cluster.points = pts;
  new_cluster.id = -1;
  new_cluster.relevance = 1;
  new_cluster.frame_id = marker.header.frame_id;
  return new_cluster;
}

void ClusterFilter::open(){
  ros::init(mlr::argc, mlr::argv, "cluster_filter", ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle;
  //sub = nh->subscribe( "/tabletop/clusters", 100, &ClusterFilter::callback, this);
  pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);
  std::cout << "Opening cluster filter. " << std::endl;
}

void ClusterFilter::close(){
  nh->shutdown();
  delete nh;
}

void ClusterFilter::step(){
  std::cout << "Step." << std::endl;

  tabletop_clusters.waitForNextRevision();

  convertMessage(tabletop_clusters.get());

  old_clusters = tracked_clusters.get();

  if ((old_clusters.size() == 0) && (raw_clusters.size() == 0))
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


void ClusterFilter::convertMessage(const visualization_msgs::MarkerArray& msg) {
  // go through currently provided clusters
  raw_clusters.clear();

  //std::cout << "Markers found: " << msg.markers.size() << std::endl;
  for(auto & marker : msg.markers){
    Cluster new_cluster = conv_Marker2Cluster(marker);
    raw_clusters.push_back(new_cluster);
  }
}


void ClusterFilter::createCostMatrix(arr& costs){

  // 3 cases:
  int num_old = old_clusters.size();
  int num_new = raw_clusters.size();
  int dims = std::max(num_old, num_new);

  if (dims == 0)
    return;

  costs = ones(dims, dims) * -1.0;
  for (int i = 0; i < num_new; i++)
  {
    for (int j = 0; j < num_old; j++)
    {
      costs(i,j) = length(raw_clusters[i].mean - old_clusters[j].mean);
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

Hungarian::~Hungarian(){}

Hungarian::Hungarian(const arr& cost_matrix)
{
  costs = cost_matrix;
  dim = costs.dim(0);
  starred = zeros(dim, dim);
  primed = starred;
  covered_rows = zeros(dim);
  covered_cols = covered_rows;
}

void Hungarian::minimize()
{
  covered_rows = covered_cols = zeros(dim);
  starred = primed = zeros(dim, dim);
  for (uint i = 0; i < dim; i++ )
  {
    double minRow = costs[i]().minIndex();
    costs[i]() -= costs(i, minRow);
  }
  costs = ~costs;

  for (uint i = 0; i < dim; i++ )
  {
    double minRow = costs[i]().minIndex();
    costs[i]() -= costs(i, minRow);
  }
  costs = ~costs;

  /*
  std::cout.precision(3);
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      std::cout << costs(i,j) << ' ';
    }
    std::cout << std::endl;
  }
  */

  starZeros();
}

void Hungarian::starZeros()
{
  for (uint i = 0; i < dim; i++ )
  {
    if (covered_rows(i))
      continue;

    for (uint j = 0; j < dim; j++ )
    {
      if (covered_cols(j))
        continue;

      if (costs(i,j) == 0)
      {
        starred(i,j) = 1;
        covered_rows(i) = 1;
        covered_cols(j) = 1;
        break;
      }
    }
  }

  covered_rows = zeros(dim);
  covered_cols = covered_rows;
  coverColumns();
}

void Hungarian::coverColumns()
{
/*  std::cout << "Cover columns, starred" << std::endl;
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      std::cout << starred(i,j);
    }
    std::cout << std::endl;
  }
  std::cout << "Primed." << std::endl;
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      std::cout << primed(i,j);
    }
    std::cout << std::endl;
  }

  std::cout << "Costs " << std::endl;
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      (costs(i,j)==0) ? std::cout << "0" : std::cout << "1";
    }
    std::cout << std::endl;
  }
  */

  uint count = 0;
  starred = ~starred;
  for (uint i = 0; i < dim; i++ )
  {
    if (sum(starred[i]()) > 0)
    {
      covered_cols(i) = 1;
      count++;
    }
  }
  starred = ~starred;

  //std::cout << "This many stars: " << count << std::endl;

  if (count == dim)
  {
    //std::cout << "Completed!\n\n\n\n\n\n" << std::endl;
    //mlr::wait();

    // finished.
    return;
  }
  prime();
}

void Hungarian::prime()
{
  /*std::cout << "Prime" << std::endl;
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      if (covered_rows(i) || covered_cols(j))
      {
        (starred(i,j) == 1) ? std::cout << '*' : std::cout << "-";
      }
      else
      {
        (costs(i,j) == 0) ? std::cout<<"0" : std::cout <<"n";
      }
    }
    std::cout << std::endl;
  }
  mlr::wait();
  */

  // Find an uncovered zero.
  for (uint i = 0; i < dim; i++ )
  {
    if (covered_rows(i))
      continue;

    for (uint j = 0; j < dim; j++ )
    {
      if (covered_cols(j))
        continue;

      if (costs(i,j) == 0)
      {
        primed(i,j) = 1;
  /*      std::cout << "Priming: " << i << ' ' << j << std::endl;
        // Check to see if there is a starred zero in this row.
        for (uint i = 0; i < dim; i++ )
        {
          for (uint j = 0; j < dim; j++ )
          {
            //(starred(i,j) == 1) ? std::cout << '*' :
            std::cout << primed(i,j);
          }
          std::cout << std::endl;
        }
        mlr::wait();
        */
        if (sum(starred[i]()) == 0)
        {
          path_row.clear();
          path_row.push_back(i);
          path_col.clear();
          path_col.push_back(j);
          //std::cout << "Path starts with: " << i << ' ' << j << std::endl;
          makePath();
          return;
        }
        else
        {
          // Cover this row
          covered_rows(i) = 1;
          // Uncover columns containing star
          uint maxIndex = starred[i]().maxIndex();
          covered_cols(maxIndex) = 0;
          prime();
          return;
        }
      }
    }
  }

  /*
  std::cout << "Done priming" << std::endl;
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      std::cout << primed(i,j);
    }
    std::cout << std::endl;
  }
  */
  // If we reach here, there are no more uncovered zeros.

  modifyCost();
  return;
}

void Hungarian::makePath()
{
  //std::cout << "Make Path" << std::endl;

  uint count = 0;

  /*
  std::cout << "Starred: " << std::endl;
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      std::cout << starred(i,j);
    }
    std::cout<<std::endl;
  }


  std::cout << "Primed: " << std::endl;
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      std::cout << primed(i,j);
    }
    std::cout<<std::endl;
  }
  */

  while (1)
  {
    //std::cout << "Count: " << count << " column: " << path_col[count] << std::endl;
    starred = ~starred;
    // find the star in the column
    int row = starred[path_col.at(count)]().maxIndex();


    //std::cout << "Star is at: " << row << ' ' << path_col[count] << std::endl;
    //mlr::wait();

    starred = ~starred;
    if (starred(row, path_col.at(count)) == 0)
    {
      //std::cout << "Done while-loop in makePath." << std::endl;
      break;
    }

    count++;
    path_row.push_back(row);
    path_col.push_back(path_col.at(count - 1));

    // find the prime in this row
    int col = primed[row]().maxIndex();
    count++;
    path_row.push_back(path_row.at(count - 1));
    path_col.push_back(col);
  }

  //std::cout << "Count is: " << count << std::endl;
  // Modify it.
  for (uint i = 0; i <= count; i++ )
  {
    uint row = path_row.at(i);
    uint col = path_col.at(i);

    if (starred(row,col))
    {
      //std::cout << "Unstarring: " << row << ' ' << col << std::endl;
      starred(row,col) = 0;
    }
    else
    {
      //std::cout << "Starring: " << row << ' ' << col << std::endl;
      starred(row,col) = 1;
    }
  }

/*  std::cout << "After modifying the path: " << std::endl;
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++ )
    {
      std::cout << starred(i,j);
    }
    std::cout<<std::endl;
  }
  mlr::wait();
*/

  // Clear covers and primes, call cover columns.
  covered_rows = covered_cols = zeros(dim);
  primed = zeros(dim, dim);
  coverColumns();
}

void Hungarian::modifyCost()
{
 // std::cout << "Modify cost: ";

  auto minCost = max(costs);
  for (uint i = 0; i < dim; i++ )
  {
    if (covered_rows(i))
      continue;

    for (uint j = 0; j < dim; j++)
    {
      if (covered_cols(j))
        continue;

      if (costs(i,j) < minCost)
        minCost = costs(i,j);
    }
  }
  //std::cout << minCost << std::endl;

  // Modify the costs
  for (uint i = 0; i < dim; i++ )
  {
    for (uint j = 0; j < dim; j++)
    {
      if (covered_rows(i))
      {
        costs(i,j) += minCost;
      }
      else if (!covered_cols(j))
      {
        costs(i,j) -= minCost;
      }
    }
  }

  prime();
}

/*
void ClusterFilter::HungarianAlgorithm(arr& costs){
  // New clusters are in raw_clusters
  // Old clusters are in tracked_clusters.get access
  // Costs are in costs.

  if (min(costs) == max(costs) )
  {
    //tracked_clusters.writeAccess();
    //tracked_clusters.set() = raw_clusters;
    //tracked_clusters.deAccess();
    return;
  }

  // First step: zero out rows
  uint dim = costs.dim(0);

  //std::cout.precision(4);
  //std::cout << "costs ---------------\n";

  for (int i = 0; i < dim; i++)
  {
    for (int j = 0; j < dim; j++)
    {
      std::cout << costs(i,j) << '\t';
    }
    std::cout << endl;
  }


  for (uint i = 0; i < dim; i++)
  {
    double row_minimum = min(costs[i]());
    costs[i]() -= row_minimum;
  }

  costs = ~costs;
  for (uint i = 0; i < dim; i++)
  {
    double cols_minimum = min(costs[i]());
    costs[i]() -= cols_minimum;
  }
  costs = ~costs;

  std::cout << "------------------\n\n\n"<< std::endl;
  for (uint i = 0; i < dim; i++)
  {
    for (uint j = 0; j < dim; j++)
    {
      std::cout << costs(i,j) << '\t';
    }
    std::cout << endl;
  }
  std::cout << "------------------\n\n\n"<< std::endl;
  for (uint i = 0; i < dim; i++)
  {
    for (uint j = 0; j < dim; j++)
    {
      (costs(i,j) == 0)? std::cout << "0" : std::cout << "1";
    }
    std::cout << endl;
  }

  arr recurse_costs = costs;
  bool finished = recurse(recurse_costs, 0);
  if (finished)
  {
    std::cout << "Finished!" << std::endl;
    costs = recurse_costs;
    mlr::wait();
  }
  std::cout << "Finished? : " << finished << std::endl;

  while( !finished )
  {
    arr costs_copy = costs;
    arr assigned = zeros(costs.dim(0), costs.dim(1));

    // Assign tasks, mark rows
    arr marked_rows = zeros(dim);
    arr marked_cols = marked_rows;
    for (uint i = 0; i < dim; i++ )
    {
      uint index = costs_copy[i]().minIndex();
      //std::cout << "Row: " << i << " at: " << index << " min_cost: " << costs_copy(i, index) << std::endl;
      if (costs_copy(i, index) != 0)
      {
        //std::cout << "Marking row: " << i << std::endl;
        //markCols(costs, assigned, marked_rows, marked_cols, i);
        continue;
      }

      assigned(i, index) = 1;

      for (uint j = 0; j < dim; j++)
      {
        if (( costs_copy(i,j) == 0 ) && ( j != index ))
        {
          costs_copy(i, j) = 5;
        }
        else if ((costs_copy(j, index) == 0) && (j != i ))
        {
          costs_copy(j, index) = 5;
        }
      }
    }

    for (uint i = 0; i < dim; i++)
    {
      uint max_index = assigned[i]().maxIndex();
      if (assigned(i, max_index) == 0)
      {
        marked_rows(i) = 1;
        std::cout << "Marking cols from row: " << i << std::endl;
        markCols(costs, assigned, marked_rows, marked_cols, i);
      }
    }


    for (uint i = 0; i < dim; i++)
      std::cout << i << ' ' << marked_rows(i) << ' ' << marked_cols(i) << std::endl;

    for (uint i = 0; i < dim; i++)
    {
      for (uint j = 0; j < dim; j++ )
      {
        if (!marked_rows(i) && marked_cols(j))
          std::cout << "+";
        else if (!marked_rows(i))
          std::cout << "-";
        else if (marked_cols(j))
          std::cout << "|";
        else
          std::cout << "0";
      }
      std::cout << std::endl;
    }

    uint count = 0;
    for (uint i = 0; i < dim; i++)
    {
      count += (marked_rows(i) + marked_cols(i));
    }
    std::cout << "Count: " << count << "  dim: " << dim << std::endl;

    if (count >= dim )
    {
      arr recurse_test = costs;
      finished = recurse(recurse_test, 0, 1);
      std::cout << "Finished? : " << finished << std::endl;
      mlr::wait();
    }

    // Now we should have the minimum spanning number of lines.
    // Now we need to find the lowest element of those left.
    // Lowest element in marked rows and unmarked columns
    double min_element = DBL_MAX;
    for (uint i = 0; i < dim; i++)
    {
      // Only marked rows
      if (!marked_rows(i))
        continue;

      for (uint j = 0; j < dim; j++ )
      {
        // Only unmarked columns
        if (marked_cols(j))
          continue;

        //std::cout << "Minimum: " << i << ' ' << j << ' ' << costs(i,j) << std::endl;

        if ( min_element > costs(i,j) )
          min_element = costs(i,j);
      }
    }

    std::cout << "Min element: " << min_element << std::endl;
    // Subtract min element from every unmarked element, add to every element covered by 2 lines.
    for (uint i = 0; i < dim; i++)
    {
      for (uint j = 0; j < dim; j++ )
      {
        if ( (!marked_rows(i)) && (marked_cols(j)) )
        {
          //std::cout << "ij increasing: " << i << ' ' << j << std::endl;
          costs(i,j) += min_element;
        }
        else if ( (marked_rows(i)) && (!marked_cols(j)) )
        {
          //std::cout << "ij decreasing: " << i << ' ' << j << std::endl;
          costs(i,j) -= min_element;
        }
      }
    }
    std::cout << "\n\nNew costs " << std::endl;
    for (uint i = 0; i < dim; i++)
    {
      for (uint j = 0; j < dim; j++)
      {
        std::cout << costs(i,j) << '\t'; // << costs(i,j) << '\t';
        //(costs(i,j) == 0)? std::cout << "0" : std::cout << "1";
      }
      std::cout << endl;
    }

    std::cout << "\n\nNew costs " << std::endl;
    for (uint i = 0; i < dim; i++)
    {
      for (uint j = 0; j < dim; j++)
      {
        //std::cout << costs(i,j) << '\t'; // << costs(i,j) << '\t';
        (costs(i,j) == 0)? std::cout << "0" : std::cout << "1";
      }
      std::cout << endl;
    }
    arr recurse_test = costs;
    finished = recurse(recurse_test, 0);
    std::cout << "Finished? : " << finished << std::endl;
    for (int i = 0; i < dim; i++)
    {
      for (int j = 0; j < dim; j++)
      {
        (recurse_test(i,j)==0)? std::cout << 0 : std::cout << 1;
      }
      std::cout << endl;
    }
    mlr::wait();
  }
}


bool ClusterFilter::recurse(arr& costs, const uint row, const bool verbose){
  // find first zero in that row
  uint index = costs[row]().minIndex();
  uint dim = costs.dim(0);

  if (verbose)
  {
    std::cout << "Recurse: " << row << std::endl;
    for (uint i = 0; i < dim; i++)
    {
      for (uint j = 0; j < dim; j++)
      {
        (costs(i,j)==0)? std::cout << 0 : std::cout << 1;
      }
      std::cout << endl;
    }
    mlr::wait();
  }

  if (costs(row,index) > 0)
  {
    if (verbose)
      std::cout << "Nonzero row." << std::endl;
    return false;
  }

  //
   // So there is a zero in this row.
  //  Now there are 3 cases:
  // 1) we are at the final row, and found a case.
  // 2) we are not at the final row, but can recurse further with this zero
  // 3) we are not at the final row, and cannot recurse further with this zero
  //
 // uint dim = costs.dim(0);

  // Try recursing further.
  arr costs_copy = costs;
  for (uint i = 0; i < dim; i++)
  {
    if ((costs_copy(row, i) == 0) && (i != index))
    {
      // Setting the cost to be nonzero
      costs_copy(row, i) = 5;
    }
    else if ((costs_copy(i, index) == 0) && (i != row))
    {
      // Setting the cost to be nonzero
      costs_copy(i, index) = 5;
    }
  }

  if (verbose)
  {
    std::cout << "Modified: " << row << std::endl;
    for (uint i = 0; i < dim; i++)
    {
      for (uint j = 0; j < dim; j++)
      {
        (costs_copy(i,j)==0)? std::cout << 0 : std::cout << 1;
      }
      std::cout << endl;
    }
    mlr::wait();
  }

  if ( row == (dim - 1) )
  {
    costs = costs_copy;
    //std::cout << "Should be true.." << std::endl;
    //mlr::wait();
    return true;
  }

  // Okay, we aren't at the end. Try recursing further.

  if ( recurse(costs_copy, row + 1, verbose) )
  {
    costs = costs_copy;
    return true;
  }

  // This value doesn't work as zero, so set it to nonzero.
  costs(row, index) = 5;
  return recurse(costs, row, verbose);
}

void ClusterFilter::markCols(const arr& costs, const arr& assigned, arr& marked_rows, arr& marked_cols, const int row )
{
  for (uint i=0; i < marked_cols.dim(0); i++)
  {
    if (costs(row,i) == 0)
    {
      if (marked_cols(i) == 0)
      {
        marked_cols(i) = 1;
        std::cout << "Marking col: " << i << std::endl;
        markRows(costs, assigned, marked_rows, marked_cols, i );
      }
    }
  }
}

void ClusterFilter::markRows(const arr& costs, const arr& assigned, arr& marked_rows, arr& marked_cols, const int col )
{
  for (uint i=0; i < marked_rows.dim(0); i++)
  {
    if (costs(i, col) == 0)
    {
      if ((marked_rows(i) == 0) && (assigned(i, col) == 1))
      {
        marked_rows(i) = 1;
        std::cout << "Marking row: " << i << std::endl;
        markCols(costs, assigned, marked_rows, marked_cols, i );
      }
    }
  }
}
*/

/*
//HungarianAlgorithm(costs);

Hungarian ha(costs);
ha.minimize();

std::cout << "I have: " << ha.starred.dim(0) << " matches." << std::endl;

std::vector<Cluster> new_tracks;

uint num_old = tracked_clusters_.size();
uint num_new = raw_clusters.size();

//std::cout << num_old << ' ' << num_new << std::endl;
for (uint i = 0; i < ha.starred.dim(0); i++ )
{
  // 3 cases:
  // 1) Existed before, no longer exists. If i > num_new
  // 2) Didn't exist before. This happens iff col >= num_old
  // 3) Existed before and still exists.

  if ( i >= num_new )
  {
    //std::cout << "Doing nothing. i >= num_new  "  << i << ' ' << num_new << std::endl;
    continue;
  }

  uint col = ha.starred[i]().maxIndex();

  if ( col >= num_old )
  {
    //std::cout << "Didn't exist before. Col >= num_old: " << col << ' ' << num_old << std::endl;
  }

  //std::cout << "Assigning - New at:" << i << "  Matches old: " << col << std::endl;
  new_tracks.push_back(raw_clusters.at(i));
  //new_tracks.at(i) = raw_clusters.at(i);
  //new_tracks.at(i) = raw_clusters.at(col);
}

std::cout << "Done." <<  std::endl;
std::cout << tracked_clusters_.size() << std::endl;
tracked_clusters_ = new_tracks;
std::cout << tracked_clusters_.size() << std::endl;
std::cout << "Assigned. " << std::endl;

//tracked_clusters.writeAccess();
//tracked_clusters() = tracked_clusters_;
//tracked_clusters.deAccess();

//mlr::wait();
    // find matching cluster
    bool found = false;
    for(auto & cluster : tracked_clusters){
        if(length(mean - cluster.mean) < threshold){
            found = true;
            cluster.mean = mean;
            cluster.points = pts;
            cluster.active = true;
            cluster.relevance = 1;
            cluster.frame_id = marker.header.frame_id;
            id = cluster.id;
            break;
        }
    }
    // new cluster if not found
    if(!found){
        id = 1;
        while(id_set.find(id)!=id_set.end()) ++id;
        id_set.insert(id);
        tracked_clusters.push_back(Cluster(id, mean, pts, 1, true, marker.header.frame_id));
    }

}

// show clusters (old and new)
auto cluster_it = tracked_clusters.begin();
while(cluster_it != tracked_clusters.end()) {
    // update cluster relevance
    cluster_it->relevance *= relevance_decay_factor;
    // remove clusters with low relevance
    if(cluster_it->relevance<relevance_threshold) {
        auto next_it = cluster_it;
        ++next_it;
        id_set.erase(cluster_it->id);
        tracked_clusters.erase(cluster_it);
        cluster_it = next_it;
        break;
    }
    // make all cluster inactive
    cluster_it->active = false;

    visualization_msgs::Marker new_marker;
    new_marker.type = visualization_msgs::Marker::POINTS;
    new_marker.points = conv_arr2points(cluster_it->points);
    new_marker.scale.x = .001;
    new_marker.scale.y = .001;
    new_marker.id = cluster_it->id;
    new_marker.lifetime = ros::Duration(0.5);
    new_marker.header.stamp = ros::Time(0.);
    new_marker.header.frame_id = cluster_it->frame_id;

    new_marker.color.a = 1.0; // Don't forget to set the alpha!
    new_marker.color.r = (double)((new_marker.id*10000)%97)/97;
    new_marker.color.g = (double)((new_marker.id*10000)%91)/91;
    new_marker.color.b = (double)((new_marker.id*10000)%89)/89;
    marker_array.markers.push_back(new_marker);

    ++cluster_it;
}

cout <<"#clusters = " <<tracked_clusters.size() <<endl;
cout <<"#id = " << id_set.size() << " (max: " << *(id_set.end()) << ")" <<endl;

//pub.publish(marker_array);
*/
//}


REGISTER_MODULE(ClusterFilter)

