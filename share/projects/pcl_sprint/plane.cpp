#include "plane.h"

void passthroughFilter(pcl::PointCloud<PoinT2>::Ptr inCloud,pcl::PointCloud<PoinT2>::Ptr outCloud,double limit)
{
  pcl::PassThrough<PoinT2> pass;
  pass.setInputCloud (inCloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, limit);
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

