#include "Filtering.h"


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

Hungarian::Hungarian(const arr& cost_matrix)
{
  costs = cost_matrix;
  dim = costs.dim(0);
  starred = zeros(dim, dim);
  primed = starred;
  covered_rows = zeros(dim);
  covered_cols = covered_rows;
}

Hungarian::~Hungarian(){}

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

  if (count == dim)
  {
    return;
  }
  prime();
}

void Hungarian::prime()
{
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
        // Check to see if there is a starred zero in this row.

        if (sum(starred[i]()) == 0)
        {
          path_row.clear();
          path_row.push_back(i);
          path_col.clear();
          path_col.push_back(j);
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
  modifyCost();
  return;
}

void Hungarian::makePath()
{
  uint count = 0;

  while (1)
  {
    starred = ~starred;
    // find the star in the column
    int row = starred[path_col.at(count)]().maxIndex();

    starred = ~starred;
    if (starred(row, path_col.at(count)) == 0)
    {
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

  // Modify it.
  for (uint i = 0; i <= count; i++ )
  {
    uint row = path_row.at(i);
    uint col = path_col.at(i);

    if (starred(row,col))
    {
      starred(row,col) = 0;
    }
    else
    {
      starred(row,col) = 1;
    }
  }

  // Clear covers and primes, call cover columns.
  covered_rows = covered_cols = zeros(dim);
  primed = zeros(dim, dim);
  coverColumns();
}

void Hungarian::modifyCost()
{
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
