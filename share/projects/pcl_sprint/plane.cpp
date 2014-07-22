
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;


void passthroughFilter(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<PointT>::Ptr outCloud,double limit)
{
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (inCloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, limit);
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
}

void planeDetector(pcl::PointCloud<PointT>::Ptr inCloud,pcl::PointCloud<pcl::Normal>::Ptr inCloudNormal, pcl::ModelCoefficients::Ptr outCoefficients, pcl::PointIndices::Ptr outInliersPlane)

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

