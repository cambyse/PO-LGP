#include "plane.h"

#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

PlaneDetector::PlaneDetector(pcl::PointCloud<PointT>::Ptr _cloud) {
  cloud = _cloud;
}

void PlaneDetector::passthroughFilter(double limit)
{
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, limit);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
}

void PlaneDetector::estimateNormals(int knn)
{
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (knn);
  ne.compute (*cloud_normals);
}

pcl::ModelCoefficients PlaneDetector::detectPlane() {

  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  return *coefficients_plane;
}



