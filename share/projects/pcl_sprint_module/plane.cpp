#include "plane.h"

void extractPlanes(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, std::vector<pcl::ModelCoefficients::Ptr> &outCoefficients, std::vector<pcl::PointIndices::Ptr> &outInliers , uint numPlanes)
{
  pcl::PointCloud<PointT>::Ptr cloud_be(inCloud);
  for (uint i = 0;i<numPlanes;i++) {
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    planeDetector(cloud_be,coefficients_plane,inliers_plane);
    substractPlane(cloud_be,inliers_plane,outCloud);

    outCoefficients.push_back(coefficients_plane);
    outInliers.push_back(inliers_plane);

    cloud_be = outCloud;
  }
  cout << "Number of points after normal extraction: " << outCloud->size() << endl;
}


void passthroughFilter(pcl::PointCloud<PointT>::Ptr inCloud, pcl::PointCloud<PointT>::Ptr outCloud, double minLimit, double maxLimit)
{
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (inCloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (minLimit, maxLimit);
  pass.filter (*outCloud);
  std::cerr << "PointCloud after passthroughFilter: " << outCloud->points.size () << " data points." << std::endl;
}

void normalEstimator(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr outCloud,int knn)
{
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (inCloud);
  ne.setKSearch (knn);
  ne.compute (*outCloud);
  std::cerr << "Normal estimation completed" << std::endl;
}

void planeDetector(pcl::PointCloud<PointT>::Ptr inCloud, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane)

{
  pcl::SACSegmentation<PointT> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (200);
  seg.setDistanceThreshold (0.05);
  seg.setInputCloud (inCloud);
  // Obtain the plane inliers and coefficients
  seg.segment (*outInliersPlane, *outCoefficients);
  std::cerr << "Plane coefficients: " << *outCoefficients << std::endl;
}

void planeDetectorWithNormals(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane)

{
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

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

void substractPlane(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointIndices::Ptr inInliersPlane, pcl::PointCloud<PointT>::Ptr outCloud) {
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (inCloud);
  extract.setIndices (inInliersPlane);
  extract.setNegative (true);
  extract.filter (*outCloud);
}
