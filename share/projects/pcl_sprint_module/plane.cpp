#include "plane.h"

// void extractPlanes(pcl::PointCloud<PoinT2>::Ptr inCloud, pcl::PointCloud<PoinT2>::Ptr outCloud, uint nplanes)
// {
//   pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
//   normalEstimator(inCloud,normal_cloud,50);

//   pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//   for (uint i;i< nplanes; i++)
//   {
//   pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
//   planeDetector(inCloud,normal_cloud,coefficients_plane,inliers_plane);

//   pcl::PointCloud<PointT>::Ptr cloud_substracted_plane(new pcl::PointCloud<PointT>);
//   substractPlane(inCloud,inliers_plane,cloud_substracted_plane);
//   }

// }

void passthroughFilter(pcl::PointCloud<PoinT2>::Ptr inCloud, pcl::PointCloud<PoinT2>::Ptr outCloud, double minLimit, double maxLimit)
{
  pcl::PassThrough<PoinT2> pass;
  pass.setInputCloud (inCloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (minLimit, maxLimit);
  pass.filter (*outCloud);
  std::cerr << "PointCloud after passthroughFilter: " << outCloud->points.size () << " data points." << std::endl;
}

void normalEstimator(pcl::PointCloud<PoinT2>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr outCloud,int knn)
{
  pcl::search::KdTree<PoinT2>::Ptr tree (new pcl::search::KdTree<PoinT2> ());
  pcl::NormalEstimation<PoinT2, pcl::Normal> ne;
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (inCloud);
  ne.setKSearch (knn);
  ne.compute (*outCloud);
  std::cerr << "Normal estimation completed" << std::endl;
}

void planeDetector(pcl::PointCloud<PoinT2>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane)

{
  pcl::SACSegmentationFromNormals<PoinT2, pcl::Normal> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (inCloud);
  seg.setInputNormals (inCloudNormal);
  // Obtain the plane inliers and coefficients
  seg.segment (*outInliersPlane, *outCoefficients);
  std::cerr << "Plane coefficients: " << *outCoefficients << std::endl;
}

void substractPlane(pcl::PointCloud<PoinT2>::Ptr inCloud,pcl::PointIndices::Ptr inInliersPlane, pcl::PointCloud<PoinT2>::Ptr outCloud) {
  pcl::ExtractIndices<PoinT2> extract;
  extract.setInputCloud (inCloud);
  extract.setIndices (inInliersPlane);
  extract.setNegative (true);
  extract.filter (*outCloud);
}
