#include "object.h"


void voxelFilter(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<PointT>::Ptr outCloud, double leafSize)
{
    pcl::VoxelGrid<PointT> vg;
    //pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (inCloud);
    vg.setLeafSize (leafSize,leafSize,leafSize);
    vg.filter (*outCloud);
    std::cout << "PointCloud after filtering has: " << outCloud->points.size ()  << " data points." << std::endl; //*

}

void clusterObject(pcl::PointCloud<PointT>::Ptr cloud_filtered, int numCluster, std::vector<pcl::PointCloud<PointT>::Ptr>& list_extracted_cloud, int minPoints, int maxPoints)
{
    std::vector<pcl::PointIndices> cluster_indices;

    // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
      tree->setInputCloud (cloud_filtered);

      //std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (minPoints);
      ec.setMaxClusterSize (maxPoints);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_filtered);
      ec.extract (cluster_indices);


      int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          list_extracted_cloud.push_back(cloud_cluster);

          j++;
          if(j == numCluster) break;
        }


}

